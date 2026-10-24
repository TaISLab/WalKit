"""Colour scales for the markers. Pure python (no matplotlib: the old plot nodes pulled it in only for `jet`)."""


def _clip(v):
    return max(0.0, min(1.0, v))


def jet(value):
    """(r, g, b) in 0..1 of the classic 'jet' scale for `value` in 0..1 (blue = low, red = high)."""
    v = _clip(value)
    return (_clip(1.5 - abs(4.0 * v - 3.0)), _clip(1.5 - abs(4.0 * v - 2.0)), _clip(1.5 - abs(4.0 * v - 1.0)))


def red_to_green(value):
    """(r, g, b): red at 0, green at 1 (a stability index: bad to good)."""
    v = _clip(value)
    return (1.0 - v, v, 0.0)
