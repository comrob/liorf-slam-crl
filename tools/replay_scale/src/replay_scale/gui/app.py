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
                        help="Run directory or scale_replay_frames.csv to open at startup. "
                             "Omit to use replay_scale_tool.input_path from the config, and "
                             "the newest run under the base directory if that is empty too.")
    parser.add_argument("--base-dir", default="",
                        help="Base directory holding run folders. Overrides "
                             "replay_scale_tool.base_dir from the config.")
    parser.add_argument("--ros-params-yaml", default="",
                        help="Configuration to start from (default: bundled "
                             "config/default.yaml). Another can be loaded from "
                             "the Configuration dock at any time.")
    args = parser.parse_args(argv)

    # The config the replay will load, read here for where to look as well: the
    # run list and the startup run must agree with the CLI's.
    from ..io.paths import DEFAULT_BASE_DIR, expand_path
    from ..settings import DEFAULT_CONFIG_PATH, load_config

    config_path = expand_path(args.ros_params_yaml) if args.ros_params_yaml else DEFAULT_CONFIG_PATH
    try:
        settings, _ = load_config(config_path)
    except Exception as exc:
        print(f"Could not load {config_path}: {type(exc).__name__}: {exc}", file=sys.stderr)
        return 1
    base_dir = args.base_dir or settings.base_dir or DEFAULT_BASE_DIR
    initial_input = args.input or settings.input_path

    try:
        from PySide6.QtWidgets import QApplication
    except ImportError:
        print(_MISSING_QT, file=sys.stderr)
        return 1

    from .main_window import MainWindow

    app = QApplication(sys.argv[:1])
    window = MainWindow(base_dir=base_dir, initial_input=initial_input,
                        config_path=config_path)
    window.show()
    return app.exec()


if __name__ == "__main__":
    raise SystemExit(main())
