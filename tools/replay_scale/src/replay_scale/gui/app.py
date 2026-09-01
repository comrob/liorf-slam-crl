#!/usr/bin/env python3
"""Entry point for ``replay-scale-gui``.

A viewer, not a second way to run a replay: it calls the same
``pipeline.run_replay`` the CLI does, so it cannot do anything the CLI cannot.
"""

import argparse
import os
import sys

DESCRIPTION = """Anchor-frame vector viewer for replay_scale.

Loads a run, replays it in memory (nothing is written), and lets you scrub
frames. Each frame is drawn in the anchor body frame: the complementary
displacement from the anchor origin, and the degeneracy spaces through the
latest LiDAR position.

Selecting a run in the list *previews* it -- what it recorded, and your
reference trajectory against it -- without running the estimator; replaying is a
button away. What you apply to a run is remembered per run, so switching runs
neither loses your settings nor spreads them.

It reopens where it left off: the configuration last applied and the run it was
applied to are remembered, and restored unless an argument below says
otherwise. "Reset defaults" in the Configuration dock forgets them.
"""

_MISSING_QT = ("PySide6 is required for the GUI. Install it with:\n"
               "    pip install 'replay-scale[gui]'\n"
               "The CLI (replay-scale-trajectory) does not need it.")


def _resolvable(input_path, base_dir):
    """Whether a run path still exists, absolute or relative to the base dir."""
    expanded = os.path.expanduser(input_path)
    return (os.path.exists(expanded)
            or os.path.exists(os.path.join(os.path.expanduser(base_dir), expanded)))


def resolve_startup(input_path="", base_dir="", ros_params_yaml="", use_session=True):
    """What the viewer opens with: ``(config_path, base_dir, initial_input)``.

    Precedence, applied to each independently: an explicit argument, then the
    remembered session, then the configuration file, then the built-in default.
    An explicit ``--ros-params-yaml`` is exactly that -- not what was last
    applied -- so it takes the session out of the picture entirely.

    A remembered run that is no longer on disk is dropped rather than opened
    on: the viewer should come up on the newest run, not on an error dialog.
    Raises whatever :func:`load_config` raises for a file it cannot read.
    """
    # Imported here rather than at module scope: the entry point stays cheap
    # for --help and for the missing-Qt message.
    from ..io.paths import DEFAULT_BASE_DIR, expand_path
    from ..settings import DEFAULT_CONFIG_PATH, load_config
    from .session import load_session, session_path

    remembered = load_session() if use_session and not ros_params_yaml else None
    if remembered is not None:
        config_path, settings = session_path(), remembered[0]
    else:
        config_path = expand_path(ros_params_yaml) if ros_params_yaml else DEFAULT_CONFIG_PATH
        settings, _ = load_config(config_path)

    resolved_base = base_dir or settings.base_dir or DEFAULT_BASE_DIR
    initial_input = input_path or settings.input_path
    if initial_input and not _resolvable(initial_input, resolved_base):
        initial_input = ""
    return config_path, resolved_base, initial_input


def main(argv=None):
    parser = argparse.ArgumentParser(description=DESCRIPTION,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("input", nargs="?", default="",
                        help="Run directory or scale_replay_frames.csv to open at startup. "
                             "Omit to use the run remembered from the last session, then "
                             "replay_scale_tool.input_path from the config, and the newest "
                             "run under the base directory if that is empty too.")
    parser.add_argument("--base-dir", default="",
                        help="Base directory holding run folders. Overrides "
                             "replay_scale_tool.base_dir from the config.")
    parser.add_argument("--ros-params-yaml", default="",
                        help="Configuration to start from (default: the remembered "
                             "session, else the bundled config/default.yaml). Another "
                             "can be loaded from the Configuration dock at any time. "
                             "Giving one here ignores the remembered session.")
    parser.add_argument("--no-session", action="store_true",
                        help="Ignore the remembered configuration and run for this "
                             "launch, without forgetting them.")
    parser.add_argument("--forget-session", action="store_true",
                        help="Delete the remembered session and start as if this were "
                             "the first launch. Same as \"Reset defaults\" in the "
                             "Configuration dock, which also forgets the run on screen.")
    parser.add_argument("--forget-runs", action="store_true",
                        help="Forget what was applied to every run, so each opens with "
                             "what its config files say. The runs themselves are "
                             "untouched; only the viewer's memory of them is dropped.")
    args = parser.parse_args(argv)

    # The config the replay will load, read here for where to look as well: the
    # run list and the startup run must agree with the CLI's.
    from .run_state import forget_all_runs, runs_dir
    from .session import clear_session, session_path

    if args.forget_session:
        print(f"Forgot {session_path()}" if clear_session()
              else f"Nothing remembered at {session_path()}")
    if args.forget_runs:
        forgotten = forget_all_runs()
        print(f"Forgot {forgotten} run(s) under {runs_dir()}" if forgotten
              else f"Nothing remembered under {runs_dir()}")

    try:
        config_path, base_dir, initial_input = resolve_startup(
            input_path=args.input, base_dir=args.base_dir,
            ros_params_yaml=args.ros_params_yaml,
            use_session=not (args.no_session or args.forget_session))
    except Exception as exc:
        source = args.ros_params_yaml or "the configuration"
        print(f"Could not load {source}: {type(exc).__name__}: {exc}", file=sys.stderr)
        return 1

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
