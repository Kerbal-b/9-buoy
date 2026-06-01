from pathlib import Path
import sys

SCRIPT_DIR = Path(__file__).resolve().parent
if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))

from station.app import run as legacy_run
from station.app import parse_args as legacy_parse_args


if __name__ == "__main__":
    args = legacy_parse_args()
    if getattr(args, "debug_controller", False):
        legacy_run()
    else:
        try:
            from station.qt_app import run as qt_run
        except ModuleNotFoundError as exc:
            if exc.name == "PySide6":
                print("PySide6 is not installed. Falling back to the legacy Pygame control station.")
                legacy_run()
            else:
                raise
        else:
            qt_run(args)
