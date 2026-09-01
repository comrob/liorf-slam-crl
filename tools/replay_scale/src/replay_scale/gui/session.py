"""What the viewer remembers between launches: the last configuration that ran
and the run it ran on.

The session file is an ordinary config file -- exactly what "Save YAML…"
writes, with ``input_path`` and ``base_dir`` filled in -- so restoring a session
is just loading a config, and the file can be handed straight to the CLI as
``--ros-params-yaml``. There is deliberately no second format here to keep in
step with :mod:`replay_scale.settings`.

What it deliberately does *not* remember is the run-scoped part of a
configuration -- see :data:`~replay_scale.settings.RUN_SCOPED_SECTIONS`. The
session is the configuration carried to whatever run is opened next, and a
reference trajectory belongs to the run it was recorded with: remembering one
here would redraw it beside every other run. It is kept in the run's own
``replay_scale.yaml`` instead ("Save to run" in the viewer), so switching runs
shows that run's reference, or none.

Only the viewer reads it. A CLI run depends on nothing but its arguments and
the config file it was given, which is what makes a replay reproducible from
what is written down rather than from what someone last clicked.

Qt-free, like ``sources.py``, so it is testable on its own.
"""

import os

from ..settings import (
    RUN_SCOPED_SECTIONS,
    load_config,
    save_config,
    without_run_scoped,
)

#: Name of the remembered-session file inside the state directory.
SESSION_NAME = "last_session.yaml"


def state_dir():
    """The XDG state directory for this tool.

    State rather than config: nobody hand-edits this file, and losing it costs
    nothing but the convenience it exists for.
    """
    base = (os.environ.get("XDG_STATE_HOME")
            or os.path.join(os.path.expanduser("~"), ".local", "state"))
    return os.path.join(base, "replay_scale")


def session_path():
    """Where the remembered session is kept."""
    return os.path.join(state_dir(), SESSION_NAME)


def load_session(path=None):
    """``(settings, params)`` from the remembered session, or ``None``.

    Never raises. A missing file is the normal first launch and a corrupt one
    is not worth refusing to start over -- either way the caller falls back to
    the configuration it would have used had nothing been remembered.
    """
    path = path or session_path()
    if not os.path.isfile(path):
        return None
    try:
        settings, params = load_config(path)
    except Exception:
        return None
    # Dropped on the way in as well as on the way out: a file written before
    # the reference became run-scoped, or hand-edited since, must not put one
    # back into every run opened this launch.
    cleaned = without_run_scoped(settings)
    if any(getattr(settings, name) != getattr(cleaned, name)
           for name in RUN_SCOPED_SECTIONS):
        # Rewritten, not merely ignored: the viewer re-reads this file as the
        # base for every load, not only for the first one, so a stale reference
        # left in it would come back on the next run opened.
        save_session(cleaned, params, path=path)
    return cleaned, params


def save_session(settings, params, *, input_path="", base_dir="", path=None):
    """Remember this configuration and the run it was applied to.

    ``input_path`` and ``base_dir`` override what ``settings`` carries, because
    the run on screen is usually not the one the config file named. The
    run-scoped sections are dropped: what is remembered is the configuration to
    open the *next* run with, and those describe the run this one was.

    Returns the path written, or ``None`` when it could not be: a read-only
    home is a reason to lose the convenience, not to interrupt a session.
    """
    path = path or session_path()
    remembered = without_run_scoped(settings).evolve(
        input_path=input_path or settings.input_path,
        base_dir=base_dir or settings.base_dir)
    try:
        os.makedirs(os.path.dirname(path) or ".", exist_ok=True)
        save_config(path, remembered, params)
    except OSError:
        return None
    return path


def clear_session(path=None):
    """Forget the remembered session. True when a file was actually removed."""
    path = path or session_path()
    try:
        os.remove(path)
    except OSError:
        return False
    return True
