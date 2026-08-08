"""2D line geometry shared by the estimator and the view.

One question, asked in two places: given a set of lines, each saying "the truth
lies somewhere along here", where do they agree? The anchor-frame view draws
that point; the estimator can take a scale from it. Neither should own the
answer, so it lives here.

Pure computation: no file access, no matplotlib, no Qt.
"""

from dataclasses import dataclass

import numpy as np

#: Smallest eigenvalue ratio of the normal matrix that still gives a point.
#: This only guards against lines that are parallel to numerical precision --
#: how *useful* a distant intersection is depends on what it is wanted for, so
#: that call belongs to the caller.
MIN_LINE_SPREAD = 1e-3

#: What the fit minimises over the perpendicular distances to the lines.
LINE_FIT_NORMS = ("l2", "l1")

#: IRLS settings for the L1 fit. It converges in a handful of passes; the floor
#: is relative to the residual scale, so a line the fit happens to pass through
#: exactly cannot take unbounded weight and become the whole answer.
_L1_ITERATIONS = 12
_L1_MIN_RESIDUAL = 1e-12
_L1_RESIDUAL_FLOOR_FRACTION = 1e-3

#: Consistency factor turning a median absolute deviation into a Gaussian sigma.
_MAD_TO_SIGMA = 1.4826


@dataclass
class LineFit:
    """Where a set of lines agree, and how well they agree there."""

    point: np.ndarray            # 2
    #: 2x2 covariance of ``point``, from the residual spread. None when there
    #: are too few lines for a residual to mean anything.
    covariance: np.ndarray = None
    #: Lines that carried a usable direction, i.e. that the fit actually used.
    n_lines: int = 0
    #: RMS (L2) or median absolute (L1) perpendicular distance to the lines.
    residual: float = float("nan")


def _normals(origins, directions):
    """Unit normals and their line's point, skipping directionless entries."""
    normals, points = [], []
    for p, d in zip(origins, directions):
        d = np.asarray(d, dtype=float)[:2]
        norm = float(np.linalg.norm(d))
        if norm < 1e-12:
            continue
        normals.append(np.array([-d[1], d[0]]) / norm)
        points.append(np.asarray(p, dtype=float)[:2])
    return np.asarray(normals, dtype=float), np.asarray(points, dtype=float)


def _solve(normals, points, weights, min_spread):
    """Weighted normal equations; None when the lines are too parallel."""
    weighted = normals * weights[:, None]
    A = weighted.T @ normals
    b = weighted.T @ np.einsum("ij,ij->i", normals, points)

    eigenvalues = np.linalg.eigvalsh(A)
    if eigenvalues[-1] <= 0.0 or eigenvalues[0] / eigenvalues[-1] < min_spread:
        return None, A
    return np.linalg.solve(A, b), A


def fit_lines(origins, directions, *, norm="l2", min_spread=MIN_LINE_SPREAD):
    """Fit a point to a set of lines; returns a :class:`LineFit` or None.

    Each line is a point on it and a direction. Perpendicular distance to a line
    is the component along its normal, so with normals ``n`` and points ``p`` the
    fit minimises ``sum |n.(x - p)|^k`` -- ``k = 2`` in closed form, ``k = 1`` by
    iteratively reweighted least squares.

    ``"l1"`` is worth having because these lines are not equally trustworthy: a
    frame whose degenerate direction was estimated from a poor basis contributes
    a line that is simply wrong, and a squared cost lets it pull the answer in
    proportion to how wrong it is. Absolute deviations bound each line's pull to
    one vote, in the same way a median bounds an outlier's.

    Returns None when fewer than two lines carry a direction, or when they are
    parallel to within ``min_spread`` -- the normal case in a straight tunnel,
    where the lines do still meet, but arbitrarily far away and wherever noise
    puts them.
    """
    if norm not in LINE_FIT_NORMS:
        raise ValueError(f"line fit norm must be one of {LINE_FIT_NORMS}, got {norm!r}")

    normals, points = _normals(origins, directions)
    if len(normals) < 2:
        return None

    weights = np.ones(len(normals), dtype=float)
    point, A = _solve(normals, points, weights, min_spread)
    if point is None:
        return None

    residuals = np.einsum("ij,ij->i", normals, point - points)
    if norm == "l1":
        for _ in range(_L1_ITERATIONS):
            # The IRLS weight that turns a squared cost into an absolute one.
            floor = max(_L1_MIN_RESIDUAL,
                        _L1_RESIDUAL_FLOOR_FRACTION * float(np.median(np.abs(residuals))))
            weights = 1.0 / np.maximum(np.abs(residuals), floor)
            candidate, _ = _solve(normals, points, weights, min_spread)
            if candidate is None:
                return None
            moved = float(np.linalg.norm(candidate - point))
            point = candidate
            residuals = np.einsum("ij,ij->i", normals, point - points)
            if moved < 1e-12:
                break

    # The covariance uses the *unweighted* normal matrix in both norms: it is
    # the geometry of the lines, in the units of the view, and comparing an L1
    # ellipse with an L2 one would be meaningless if one of them were scaled by
    # IRLS weights instead.
    return LineFit(point=point, n_lines=len(normals),
                   residual=_residual_scale(residuals, norm),
                   covariance=_covariance(residuals, A, norm))


def _residual_scale(residuals, norm):
    if residuals.size == 0:
        return float("nan")
    if norm == "l1":
        return float(np.median(np.abs(residuals)))
    return float(np.sqrt(np.mean(residuals ** 2)))


def _covariance(residuals, A, norm):
    """Covariance of the fitted point, from how far the lines miss it.

    ``sigma^2 A^-1`` is the textbook result for the least-squares fit, with two
    parameters estimated from ``n`` residuals: ``A`` says how well the line
    directions pin each axis down, ``sigma`` how far the lines miss the point.
    For L1 the same form is used with a MAD-based sigma, so one wild line widens
    the ellipse as little as it moved the point. That is an approximation rather
    than an exact sampling covariance, and it is deliberately built on the
    unweighted ``A`` so the two norms' ellipses are in the same units.
    """
    n = residuals.size
    if n < 3:
        return None            # two lines meet exactly; there is no residual
    if norm == "l1":
        sigma = _MAD_TO_SIGMA * float(np.median(np.abs(residuals - np.median(residuals))))
    else:
        sigma = float(np.sqrt(np.sum(residuals ** 2) / (n - 2)))
    if not np.isfinite(sigma) or sigma <= 0.0:
        return None
    return (sigma ** 2) * np.linalg.inv(A)


def closest_point_to_lines(origins, directions, *, norm="l2", min_spread=MIN_LINE_SPREAD):
    """The point a set of lines agree on, or None. See :func:`fit_lines`."""
    fit = fit_lines(origins, directions, norm=norm, min_spread=min_spread)
    return None if fit is None else fit.point


def covariance_ellipse(covariance, *, n_sigma=1.0, n_points=181):
    """Points tracing the ``n_sigma`` ellipse of a 2x2 covariance.

    Returns a (n_points, 2) array, or None when the covariance is unusable.
    """
    if covariance is None:
        return None
    values, vectors = np.linalg.eigh(np.asarray(covariance, dtype=float))
    if not np.all(np.isfinite(values)) or values[0] <= 0.0:
        return None
    theta = np.linspace(0.0, 2.0 * np.pi, n_points)
    unit = np.stack([np.cos(theta), np.sin(theta)])
    return (vectors @ (n_sigma * np.sqrt(values)[:, None] * unit)).T
