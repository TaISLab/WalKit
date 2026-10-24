"""The boxes of the laser filters, read from their parameter files (the files the filters themselves are launched with, so the
outline cannot go out of date; the old plot_active_area had the numbers typed in). Pure python (PyYAML)."""
import yaml

_KEYS = ('min_x', 'max_x', 'min_y', 'max_y')


def _find(node, found):
    if isinstance(node, dict):
        if all(k in node for k in _KEYS):
            found.append(node)
        for value in node.values():
            _find(value, found)
    elif isinstance(node, list):
        for value in node:
            _find(value, found)


def boxes_from_yaml(path):
    """Every box (a mapping with min_x, max_x, min_y, max_y and, usually, box_frame) found anywhere in the YAML file ->
    [{'frame', 'min_x', 'max_x', 'min_y', 'max_y'}]."""
    with open(path, encoding='utf-8') as f:
        data = yaml.safe_load(f)
    found = []
    _find(data, found)
    return [{'frame': b.get('box_frame', 'laser'), **{k: float(b[k]) for k in _KEYS}} for b in found]
