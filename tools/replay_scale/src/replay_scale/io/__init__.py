"""File readers and writers.

Dumb serialization only: each module here turns bytes on disk into
:mod:`replay_scale.core.model` objects or back. May import ``core.model`` and
``core.se3`` for the shared vocabulary, but never ``core.estimator`` -- nothing
here decides anything about a replay.
"""
