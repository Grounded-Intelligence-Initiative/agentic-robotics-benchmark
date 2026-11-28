"""The verify stage's entry point of the grip-controller acceptance (tests/test.sh runs it as root): it puts the grading
packages (tests/grip, a path root, not a package) first on sys.path and runs grader.verify_main.main(). Everything
else lives in tests/grip (its runtime contract: tests/grip/CONTRACT.md)."""
import os
import sys

sys.dont_write_bytecode = True
_GRIP = os.path.join(os.path.dirname(os.path.abspath(__file__)), "grip")
while _GRIP in sys.path:
    sys.path.remove(_GRIP)
sys.path.insert(0, _GRIP)

from grader.verify_main import main  # noqa: E402

if __name__ == "__main__":
    main()
