"""Append-only run folders.  Reruns create new folders; nothing is pruned."""

from __future__ import annotations

import csv
from datetime import datetime, timezone
import hashlib
import json
import os
from pathlib import Path
import re
from typing import Any, Iterable, Mapping
from uuid import uuid4

from .inputs import to_jsonable


class RunStore:
    """A single immutable-snapshot run, with an append-only hallmark index."""

    def __init__(self, path: str | Path):
        self.path = Path(path).resolve()
        if not self.path.is_dir():
            raise FileNotFoundError(self.path)

    def _target(self, relative: str | Path) -> Path:
        relative = Path(relative)
        target = (self.path / relative).resolve()
        if relative.is_absolute() or not target.is_relative_to(self.path) or target == self.path:
            raise ValueError("Run output must be a relative file path inside this run")
        target.parent.mkdir(parents=True, exist_ok=True)
        return target

    def write_json(self, relative: str | Path, value: Any) -> Path:
        target = self._target(relative)
        encoded = json.dumps(to_jsonable(value), indent=2, allow_nan=False) + "\n"
        with target.open("x", encoding="utf-8", newline="\n") as stream:
            stream.write(encoded)
            stream.flush()
            os.fsync(stream.fileno())
        return target

    def write_text(self, relative: str | Path, text: str) -> Path:
        target = self._target(relative)
        with target.open("x", encoding="utf-8", newline="\n") as stream:
            stream.write(text)
            stream.flush()
            os.fsync(stream.fileno())
        return target

    def write_csv(self, relative: str | Path, fieldnames: Iterable[str],
                  rows: Iterable[Mapping[str, Any]]) -> Path:
        target = self._target(relative)
        with target.open("x", encoding="utf-8", newline="") as stream:
            writer = csv.DictWriter(stream, fieldnames=list(fieldnames))
            writer.writeheader()
            writer.writerows(rows)
            stream.flush()
            os.fsync(stream.fileno())
        return target

    def snapshot_inputs(self, data_dir: str | Path) -> dict[str, dict[str, Any]]:
        source = Path(data_dir).resolve()
        destination = self.path / "inputs"
        destination.mkdir(exist_ok=False)
        manifest = {}
        originals = sorted(source.glob("*.csv"))
        for name in ("input_assumptions.json", "study.json"):
            assumptions = source / name
            if assumptions.is_file():
                originals.append(assumptions)
        for original in originals:
            content = original.read_bytes()
            with (destination / original.name).open("xb") as stream:
                stream.write(content)
                stream.flush()
                os.fsync(stream.fileno())
            manifest[original.name] = {"sha256": hashlib.sha256(content).hexdigest(), "bytes": len(content)}
        if not manifest:
            raise ValueError(f"No input CSV files found in {source}")
        self.write_json("inputs/manifest.json", manifest)
        return manifest

    def checkpoint(self, cycle: int, payload: Any) -> Path:
        if cycle < 0:
            raise ValueError("Checkpoint cycle cannot be negative")
        return self.write_json(f"checkpoints/cycle_{cycle:06d}.json", payload)

    def append_hallmark(self, row: Mapping[str, Any]) -> Path:
        """Append one scheduled checkpoint record without replacing prior rows."""
        if not row:
            raise ValueError("A hallmark row cannot be empty")
        target = self.path / "hallmarks.csv"
        exists = target.exists()
        fieldnames = list(row)
        if exists:
            with target.open("r", encoding="utf-8", newline="") as stream:
                reader = csv.DictReader(stream)
                fieldnames = reader.fieldnames or []
                if set(fieldnames) != set(row):
                    raise ValueError("Hallmark columns must remain unchanged within a run")
                if "cycle" in fieldnames:
                    previous = list(reader)
                    if previous and int(row["cycle"]) <= int(previous[-1]["cycle"]):
                        raise ValueError("Hallmark cycle must increase; existing records cannot be replaced")
        with target.open("a" if exists else "x", encoding="utf-8", newline="") as stream:
            writer = csv.DictWriter(stream, fieldnames=fieldnames)
            if not exists:
                writer.writeheader()
            writer.writerow(row)
            stream.flush()
            os.fsync(stream.fileno())
        return target


def create_run(runs_dir: str | Path, project_id: str, data_dir: str | Path,
               metadata: Mapping[str, Any] | None = None) -> RunStore:
    """Create a uniquely named run and copy its exact original CSV inputs."""
    if not re.fullmatch(r"[A-Za-z0-9][A-Za-z0-9_.-]*", project_id):
        raise ValueError("project_id is not a safe folder name")
    now = datetime.now(timezone.utc)
    run_id = now.strftime("%Y%m%dT%H%M%S_%fZ") + "_" + uuid4().hex[:8]
    path = Path(runs_dir).resolve() / project_id / run_id
    path.mkdir(parents=True, exist_ok=False)
    store = RunStore(path)
    manifest = store.snapshot_inputs(data_dir)
    store.write_json("run.json", {
        "schema_version": 1,
        "run_id": run_id,
        "project_id": project_id,
        "created_utc": now.isoformat(),
        "input_source": str(Path(data_dir).resolve()),
        "input_sha256": {name: item["sha256"] for name, item in manifest.items()},
        "metadata": dict(metadata or {}),
        "retention": "No automatic deletion; output snapshots are exclusive writes.",
    })
    return store


def load_checkpoint(path: str | Path) -> dict[str, Any]:
    with Path(path).open("r", encoding="utf-8") as stream:
        value = json.load(stream)
    if not isinstance(value, dict):
        raise ValueError("Checkpoint must contain a JSON object")
    return value
