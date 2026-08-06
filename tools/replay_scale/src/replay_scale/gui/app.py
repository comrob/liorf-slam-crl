#!/usr/bin/env python3
"""Entry point for ``replay-scale-gui``.

A viewer, not a second way to run a replay: it calls the same
``pipeline.run_replay`` the CLI does, so it cannot do anything the CLI cannot.
"""

import argparse
import sys

DESCRIPTION = """Anchor-frame vector viewer for replay_scale.

Loads a run, replays it in memory (nothing is written), and lets you scrub
frames. Each frame is drawn in the anchor body frame: the complementary
displacement from the anchor origin, and the degenerate directions through the
latest LiDAR position.
"""

_MISSING_QT = ("PySide6 is required for the GUI. Install it with:\n"
               "    pip install 'replay-scale[gui]'\n"
               "The CLI (replay-scale-trajectory) does not need it.")


def main(argv=None):
    parser = argparse.ArgumentParser(description=DESCRIPTION,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("input", nargs="?", default="",
                        help="Run directory or scale_replay_frames.csv to open at "
                             "startup. Defaults to the newest run under --base-dir.")
    parser.add_argument("--base-dir", default="~/.ros/lili_logs",
                        help="Base directory holding run folders (default: %(default)s).")
    args = parser.parse_args(argv)

    try:
        from PySide6.QtWidgets import QApplication
    except ImportError:
        print(_MISSING_QT, file=sys.stderr)
        return 1

    from .main_window import MainWindow

    app = QApplication(sys.argv[:1])
    window = MainWindow(base_dir=args.base_dir, initial_input=args.input)
    window.show()
    return app.exec()


if __name__ == "__main__":
    raise SystemExit(main())
