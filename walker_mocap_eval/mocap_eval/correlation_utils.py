#!/usr/bin/env python3
"""Correlacion de Pearson entre dos parametros ya calculados por bag,
agregada por sujeto (igual criterio que el resto del informe: CA/LA/MF/MT,
no una unica cifra mezclando sujetos de complexion/sexo distintos) -- usado
por los puntos 2 (asimetria maneta vs asimetria cinematica) y 5
(inclinacion de tronco vs margen de estabilidad) de la ampliacion.
"""
import math
import statistics


def user_of(bag):
    return bag.split("_")[0]


def group_bags(bags):
    groups = {}
    for b in bags:
        groups.setdefault(user_of(b), []).append(b)
    return dict(sorted(groups.items()))


def pearson_r(xs, ys):
    """r de Pearson, ignorando pares con NaN; NaN si quedan <3 puntos o
    varianza nula en algun eje (statistics.correlation no esta definido en
    ese caso)."""
    pairs = [(x, y) for x, y in zip(xs, ys)
             if not (isinstance(x, float) and math.isnan(x)) and not (isinstance(y, float) and math.isnan(y))]
    if len(pairs) < 3:
        return float("nan"), len(pairs)
    xs2, ys2 = zip(*pairs)
    if len(set(xs2)) < 2 or len(set(ys2)) < 2:
        return float("nan"), len(pairs)
    return statistics.correlation(xs2, ys2), len(pairs)


def correlation_by_group(bags, x_by_bag, y_by_bag):
    """Devuelve dict {grupo: (r, n)} para cada sujeto mas 'TOTAL' -- mismo
    criterio de agrupacion que generate_latex_report.py."""
    groups = group_bags(bags)
    out = {}
    for g, gbags in groups.items():
        xs = [x_by_bag[b] for b in gbags]
        ys = [y_by_bag[b] for b in gbags]
        out[g] = pearson_r(xs, ys)
    xs_all = [x_by_bag[b] for b in bags]
    ys_all = [y_by_bag[b] for b in bags]
    out["TOTAL"] = pearson_r(xs_all, ys_all)
    return out
