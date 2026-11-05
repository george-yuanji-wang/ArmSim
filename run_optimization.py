"""Stable command-line entry point; paths are anchored to this repository.

    uv run python run_optimization.py --demo
    uv run python run_optimization.py --real
    uv run python run_optimization.py optimize --project EXAMPLE

The absolute script path also works when launched from another directory.
"""

import faulthandler
from pathlib import Path
import sys


ROOT = Path(__file__).resolve().parent


def main(argv=None):
    args = list(sys.argv[1:] if argv is None else argv)
    if args and args[0] == "--demo":
        args = ["optimize", "--data-dir", str(ROOT / "optimization/examples/synthetic"),
                "--project", "SYNTHETIC_DEMO", *args[1:]]
    elif args and args[0] == "--real":
        args = ["optimize", "--data-dir", str(ROOT / "optimization/data/real"),
                *args[1:]]
    # The script directory wins over unrelated packages named 'optimization'.
    sys.path.insert(0, str(ROOT))
    from optimization.__main__ import main as run
    return run(args)


if __name__ == "__main__":
    faulthandler.enable()
    raise SystemExit(main())
