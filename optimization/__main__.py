"""Run with ``uv run python -m optimization --help``."""

import argparse
from collections import defaultdict
from dataclasses import replace
import faulthandler
from pathlib import Path
import sys

from .inputs import DATA_DIR, InputError, load_inputs


def main(argv=None):
    parser = argparse.ArgumentParser(description="Headless arm studies with persistent CSV results.")
    sub = parser.add_subparsers(dest="command", required=True)
    for name, help_text in (
        ("validate", "Validate input CSVs without changing them."),
        ("prepare", "Snapshot inputs and write the reference-pose CSV."),
        ("optimize", "Run a finite reproducible mixed design search."),
    ):
        command = sub.add_parser(name, help=help_text)
        command.add_argument("--data-dir", type=Path, default=DATA_DIR)
        if name != "validate":
            command.add_argument("--project", help="Project ID; otherwise process every requirements row.")
            command.add_argument("--runs-dir", type=Path, default=Path(__file__).parent / "runs")
            command.add_argument("--pose-limit", type=int, help="Reduced smoke subset; omit for the full grid.")
        if name == "optimize":
            command.add_argument("--layout", type=int, choices=(7, 8, 9),
                                 help="Run only this configuration; default runs 07, 08 and 09 independently.")
            command.add_argument("--cycles", type=int, help="Override candidate-evaluation budget.")
            command.add_argument("--checkpoint-every", type=int)
            command.add_argument("--seed", type=int)
    export = sub.add_parser("export", help="Re-export a saved winner as a complete ArmSim MJCF.")
    export.add_argument("--run", type=Path, required=True, help="Existing optimization run folder.")
    export.add_argument("--output", type=Path, help="Output XML; default is a new exports folder in the run.")
    export.add_argument("--replace", action="store_true", help="Archive an existing output before replacing it.")
    args = parser.parse_args(argv)
    if args.command == "export":
        try:
            from .saved_export import export_saved_run
            target = export_saved_run(args.run, args.output, replace=args.replace)
            print(f"Complete ArmSim MJCF: {target}", flush=True)
            return 0
        except (InputError, ValueError, OSError) as exc:
            print(f"Export error: {exc}", file=sys.stderr)
            return 2
    args.data_dir = args.data_dir.resolve()
    print(f"ArmSim optimization: {args.command}", flush=True)
    print(f"Python: {sys.executable}", flush=True)
    print(f"Inputs: {args.data_dir}", flush=True)
    if args.command != "validate":
        args.runs_dir = args.runs_dir.resolve()
        print(f"Results: {args.runs_dir}", flush=True)
    try:
        inputs = load_inputs(args.data_dir, require_catalogs=args.command == "optimize")
        if args.command == "validate":
            print(f"Valid CSVs: {len(inputs.motors)} motors, {len(inputs.reducers)} reducers, "
                  f"{len(inputs.requirements)} projects, "
                  f"{len(inputs.links)} link bounds.")
            if not inputs.motors or not inputs.reducers:
                print("Catalog templates are empty. Fill them before optimizing; prepare works now.")
            return 0
        if args.project:
            projects = [r for r in inputs.requirements if r.project_id == args.project]
            if not projects:
                raise InputError(f"No project named {args.project!r} in requirement.csv")
        else:
            projects = list(inputs.requirements)
        if not projects:
            raise InputError("No projects. Add a reach/payload row to requirement.csv.")
        # Load simulation only for commands that use it; validation remains a light CSV check.
        from .search import optimize_project, prepare_project
        if args.command == "optimize":
            projects = [replace(requirement,
                                project_id=f"{requirement.project_id}_{number:02d}",
                                allowed_layouts=(number,))
                        for requirement in projects
                        for number in ((args.layout,) if args.layout is not None
                                       else requirement.allowed_layouts)]
        completed = defaultdict(list)
        failed = False
        for requirement in projects:
            if args.command == "prepare":
                path = prepare_project(inputs, requirement, args.data_dir, args.runs_dir,
                                       pose_limit=args.pose_limit)
            else:
                try:
                    path = optimize_project(inputs, requirement, args.data_dir, args.runs_dir,
                                            cycles=args.cycles, checkpoint_every=args.checkpoint_every,
                                            seed=args.seed, pose_limit=args.pose_limit)
                except (InputError, ValueError, OSError) as exc:
                    failed = True
                    print(f"Configuration search failed ({requirement.project_id}): {exc}", file=sys.stderr)
                    continue
                completed[requirement.project_id.rsplit("_", 1)[0]].append(path)
            print(f"Saved: {path}", flush=True)
            if args.command == "optimize":
                print(f"Best candidate MJCF: {path / 'best_model.xml'}", flush=True)
        for project_id, paths in completed.items():
            if len(paths) == 3:
                from .comparison import write_comparison
                comparison = write_comparison(paths, args.runs_dir / "comparisons" / project_id)
                print(f"Configuration comparison: {comparison}", flush=True)
        return 2 if failed else 0
    except (InputError, ValueError, OSError) as exc:
        print(f"Input/run error: {exc}", file=sys.stderr)
        return 2
    except KeyboardInterrupt:
        print("Interrupted. Existing run snapshots/checkpoints were retained.", file=sys.stderr)
        return 130


if __name__ == "__main__":
    faulthandler.enable()
    raise SystemExit(main())
