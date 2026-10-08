#!/usr/bin/env python3
"""Auto-test con datos sinteticos (sin bags) para correlation_utils.py."""
import math
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from mocap_eval import correlation_utils  # noqa: E402


def approx(a, b, tol=1e-6):
    return math.isfinite(a) and math.isfinite(b) and abs(a - b) <= tol


def test_pearson_perfect_linear():
    r, n = correlation_utils.pearson_r([1, 2, 3, 4], [2, 4, 6, 8])
    assert approx(r, 1.0) and n == 4, (r, n)
    r2, _ = correlation_utils.pearson_r([1, 2, 3, 4], [8, 6, 4, 2])
    assert approx(r2, -1.0), r2
    print("test_pearson_perfect_linear: OK")


def test_pearson_ignores_nan_pairs():
    r, n = correlation_utils.pearson_r([1, 2, float("nan"), 4], [2, 4, 99, 8])
    assert approx(r, 1.0) and n == 3, (r, n)
    print("test_pearson_ignores_nan_pairs: OK")


def test_group_bags_by_prefix():
    bags = ["CA_test01", "CA_test02", "LA_test1", "MF_test01"]
    groups = correlation_utils.group_bags(bags)
    assert groups == {"CA": ["CA_test01", "CA_test02"], "LA": ["LA_test1"], "MF": ["MF_test01"]}, groups
    print("test_group_bags_by_prefix: OK")


def test_correlation_by_group():
    # pearson_r exige >=3 puntos (con 2, r es siempre +/-1 o indefinido y no
    # es estadisticamente informativo) -- 3 bags por sujeto en este test.
    bags = ["CA_test01", "CA_test02", "CA_test03", "LA_test1", "LA_test2", "LA_test3"]
    x = {"CA_test01": 1, "CA_test02": 2, "CA_test03": 3, "LA_test1": 10, "LA_test2": 20, "LA_test3": 30}
    y = {"CA_test01": 2, "CA_test02": 4, "CA_test03": 6, "LA_test1": 20, "LA_test2": 40, "LA_test3": 60}
    out = correlation_utils.correlation_by_group(bags, x, y)
    assert approx(out["CA"][0], 1.0), out["CA"]
    assert approx(out["LA"][0], 1.0), out["LA"]
    assert approx(out["TOTAL"][0], 1.0), out["TOTAL"]
    print("test_correlation_by_group: OK")


if __name__ == "__main__":
    test_pearson_perfect_linear()
    test_pearson_ignores_nan_pairs()
    test_group_bags_by_prefix()
    test_correlation_by_group()
    print("\nTodos los selftests OK")
