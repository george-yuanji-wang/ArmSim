"""Regenerate a complete MJCF from an existing winner without rerunning search."""

from dataclasses import fields
from datetime import datetime, timezone
import json
import os
from pathlib import Path
from uuid import uuid4

from .design import Candidate
from .inputs import InputError, Inputs, LinkBounds, Motor, Reducer, Requirement


def _require_scored_base_geometry(run):
    """An export cannot silently reinterpret a score from an older model."""
    from .model_factory import GEOMETRY_CONTRACT

    result_path = run / "result.json"
    try:
        result = json.loads(result_path.read_text(encoding="utf-8"))
    except (OSError, ValueError) as exc:
        raise InputError(
            f"Cannot verify scored base geometry in {result_path}. "
            "Rerun optimization before exporting this winner."
        ) from exc
    summary = result.get("summary", {}) if isinstance(result, dict) else {}
    if (not isinstance(result, dict) or not isinstance(summary, dict)
            or result.get("status") != "completed"
            or summary.get("geometry_contract") != GEOMETRY_CONTRACT
            or summary.get("geometry_valid") is not True):
        raise InputError(
            f"Saved winner in {run} was not scored with valid {GEOMETRY_CONTRACT} geometry. "
            "Rerun optimization; rebuilding a legacy winner would invalidate its recorded Pmax and J_G."
        )


def load_saved_design(run_dir):
    run = Path(run_dir).resolve()
    try:
        candidate = Candidate.from_dict(json.loads((run / "best_design.json").read_text(encoding="utf-8")))
        evaluated = json.loads((run / "evaluated_inputs.json").read_text(encoding="utf-8"))
        settings = json.loads((run / "run.json").read_text(encoding="utf-8"))
    except (OSError, ValueError, TypeError, KeyError) as exc:
        raise InputError(f"Cannot read saved winner from {run}: {exc}") from exc
    allowed = {field.name for field in fields(Requirement)}

    def requirement(values):
        values = {key: value for key, value in values.items() if key in allowed}
        if "allowed_layouts" in values:
            values["allowed_layouts"] = tuple(values["allowed_layouts"])
        return Requirement(**values)

    try:
        inputs = Inputs(
            tuple(Motor(**row) for row in evaluated["motors"]),
            tuple(Reducer(**row) for row in evaluated["reducers"]),
            tuple(requirement(row) for row in evaluated["requirements"]),
            tuple(LinkBounds(**row) for row in evaluated["links"]),
        )
        resolved = requirement(settings["metadata"]["requirement"])
    except (TypeError, KeyError, ValueError) as exc:
        raise InputError(f"Saved input data is incomplete in {run}: {exc}") from exc
    return candidate, inputs, resolved


def export_saved_run(run_dir, output=None, *, replace=False):
    """Append a new export; explicit replacement archives the previous file."""
    from .export import export_model

    run = Path(run_dir).resolve()
    _require_scored_base_geometry(run)
    candidate, inputs, requirement = load_saved_design(run)
    xml = export_model(candidate, inputs, requirement)
    timestamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%S_%fZ")
    target = (Path(output).resolve() if output is not None else
              run / "exports" / f"{timestamp}_{uuid4().hex[:8]}" / "best_model.xml")
    target.parent.mkdir(parents=True, exist_ok=True)
    if target.exists():
        if not replace:
            raise InputError(f"Output already exists: {target}. Use --replace to archive and replace it.")
        archive = target.parent / "archive"
        archive.mkdir(exist_ok=True)
        backup = archive / f"{target.stem}_{timestamp}_{uuid4().hex[:8]}{target.suffix}"
        with backup.open("xb") as stream:
            stream.write(target.read_bytes())
            stream.flush()
            os.fsync(stream.fileno())
    if replace:
        temporary = target.with_name(f".{target.name}.{uuid4().hex}.tmp")
        with temporary.open("x", encoding="utf-8", newline="\n") as stream:
            stream.write(xml)
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(temporary, target)
    else:
        with target.open("x", encoding="utf-8", newline="\n") as stream:
            stream.write(xml)
            stream.flush()
            os.fsync(stream.fileno())
    return target
