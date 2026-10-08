#!/usr/bin/env python3
"""Estima el centro de gravedad (CoM) de cuerpo completo, por frame, a
partir de la postura del mocap y los datos antropometricos del usuario
(sexo -- para elegir la tabla; peso -- para dar tambien la masa absoluta de
cada segmento, aunque la POSICION del CoM no depende de el, solo de
fracciones relativas; altura -- usada como contraste/sanity-check, no para
derivar la posicion, ver mas abajo), para poder compararlo con lo que
publica el andador (`walker_centroid_support/support_centroid`,
`walker_stability/StabilityStamped`) -- paso 3 del plan.

Metodo: suma ponderada de los centros de masa de los segmentos corporales
(modelo antropometrico segmentario de De Leva 1996, "Adjustments to
Zatsiorsky-Seluyanov's segment inertia parameters", J. Biomech 29(9):1223-30
-- tabla con fracciones de masa y de posicion del CoM por segmento,
separadas por sexo, referenciadas a articulaciones en vez de puntos oseos).
Cada segmento cuyo endpoint distal SI esta entre los 13 keypoints
etiquetados (torso/hombros/codos/muñecas/caderas/rodillas/tobillos) se
calcula con su CoM real (interpolando entre las dos articulaciones segun la
fraccion de De Leva); los segmentos terminales sin marcador propio (cabeza,
manos, pies -- no hay marcador de vertice, dedos ni punta de pie en el
esquema de 13 keypoints) se aproximan con su masa concentrada en la
articulacion proximal disponible (hombro, muñeca, tobillo respectivamente)
en vez de inventar una longitud de segmento no medida. Esto sesga
ligeramente el CoM total hacia abajo/atras de su posicion real (cabeza mas
alta que el hombro, mano mas alla de la muñeca, pie mas alla del tobillo),
pero:
  - Esos tres segmentos suman solo ~8.5% de la masa corporal total (cabeza
    ~6.7-6.9%, dos manos ~1.1-1.2%, dos pies ~2.6-2.7%), asi que el sesgo en
    la posicion final es pequeño.
  - Para el uso previsto (comparar el CoM horizontal, en el plano del
    suelo, contra `support_centroid` del andador, y buscar correlacion
    entre bags/condiciones, no un valor absoluto calibrado), el sesgo
    vertical no importa y el sesgo horizontal es minimo: con el tronco
    razonablemente vertical, cabeza/manos/pies estan casi encima de sus
    articulaciones proximales en el plano X-Y de todos modos.
  - Es preferible a introducir una constante de longitud cabeza-cuello (o
    mano/pie) no medida y no verificada aqui, que aumentaria la
    incertidumbre en vez de reducirla.

Tabla De Leva (verificada contra el paper original, no de memoria):
"""
import numpy as np

# {segmento: (fraccion_masa, fraccion_CoM_desde_extremo_proximal)}
# fraccion_CoM se usa solo en los segmentos con endpoint distal medido
# (trunk, upper_arm, forearm, thigh, shank); en head/hand/foot no se usa
# (CoM aproximado en la articulacion proximal, ver docstring del modulo).
DE_LEVA_MALE = {
    "head": (0.0694, 0.5002),
    "trunk": (0.4346, 0.5138),
    "upper_arm": (0.0271, 0.5772),
    "forearm": (0.0162, 0.4574),
    "hand": (0.0061, 0.7900),
    "thigh": (0.1416, 0.4095),
    "shank": (0.0433, 0.4395),
    "foot": (0.0137, 0.4415),
}
DE_LEVA_FEMALE = {
    "head": (0.0668, 0.4841),
    "trunk": (0.4257, 0.4964),
    "upper_arm": (0.0255, 0.5754),
    "forearm": (0.0138, 0.4559),
    "hand": (0.0056, 0.7474),
    "thigh": (0.1478, 0.3612),
    "shank": (0.0481, 0.4352),
    "foot": (0.0129, 0.4014),
}

# Suma de fracciones de masa (tronco + cabeza + 2x cada segmento de
# extremidad) = 1.000 (De Leva), asi que sirven directamente como pesos de
# una media ponderada sin necesitar el peso real del usuario para la
# POSICION del CoM -- el peso solo se usa para dar tambien la masa absoluta
# (kg) de cada segmento (parametro `weight_kg`, opcional).


def _table_for(gender):
    g = gender.strip().lower()
    if g in ("f", "femenino", "female", "woman", "mujer"):
        return DE_LEVA_FEMALE
    if g in ("m", "masculino", "male", "man", "hombre"):
        return DE_LEVA_MALE
    raise ValueError(f"sexo desconocido: {gender!r} (se esperaba m/f o Masculino/Femenino)")


def _lerp(p_prox, p_dist, frac):
    return tuple(p_prox[i] + frac * (p_dist[i] - p_prox[i]) for i in range(3))


def _midpoint(p1, p2):
    return tuple((p1[i] + p2[i]) / 2.0 for i in range(3))


def estimate_com_at_frame(markers, gender, weight_kg=None):
    """markers: dict marker_name -> (x,y,z) con, como minimo, los 13
    keypoints de soma_net.schema (torso, l/r_shoulder, l/r_elbow, l/r_wrist,
    l/r_hip, l/r_knee, l/r_ankle) presentes en ESTE frame (sin huecos: el
    llamador debe emparejar por tiempo antes, ver com_series()).

    Devuelve (com_xyz, segments), donde segments es una lista de
    {"name", "mass_frac", "mass_kg" (o None si no se dio weight_kg), "pos"}
    -- por si se quiere inspeccionar/depurar segmento a segmento (p.ej.
    para ver cuanto pesa cada brazo) sin recalcular.
    """
    table = _table_for(gender)
    hip_mid = _midpoint(markers["l_hip"], markers["r_hip"])
    shoulder_mid = _midpoint(markers["l_shoulder"], markers["r_shoulder"])

    segs = []

    def add(name, mass_frac, pos):
        segs.append({
            "name": name, "mass_frac": mass_frac,
            "mass_kg": (mass_frac * weight_kg) if weight_kg is not None else None,
            "pos": pos,
        })

    # Tronco: cadera media (proximal) -> hombro medio (distal).
    mass_frac, com_frac = table["trunk"]
    add("trunk", mass_frac, _lerp(hip_mid, shoulder_mid, com_frac))

    # Cabeza+cuello: sin marcador de vertice -- aproximada en hombro medio
    # (ver docstring del modulo).
    mass_frac, _ = table["head"]
    add("head", mass_frac, shoulder_mid)

    for side in ("l", "r"):
        shoulder, elbow, wrist = markers[f"{side}_shoulder"], markers[f"{side}_elbow"], markers[f"{side}_wrist"]
        hip, knee, ankle = markers[f"{side}_hip"], markers[f"{side}_knee"], markers[f"{side}_ankle"]

        mass_frac, com_frac = table["upper_arm"]
        add(f"{side}_upper_arm", mass_frac, _lerp(shoulder, elbow, com_frac))

        mass_frac, com_frac = table["forearm"]
        add(f"{side}_forearm", mass_frac, _lerp(elbow, wrist, com_frac))

        mass_frac, _ = table["hand"]
        add(f"{side}_hand", mass_frac, wrist)  # aproximada en la muñeca

        mass_frac, com_frac = table["thigh"]
        add(f"{side}_thigh", mass_frac, _lerp(hip, knee, com_frac))

        mass_frac, com_frac = table["shank"]
        add(f"{side}_shank", mass_frac, _lerp(knee, ankle, com_frac))

        mass_frac, _ = table["foot"]
        add(f"{side}_foot", mass_frac, ankle)  # aproximada en el tobillo

    total_mass_frac = sum(s["mass_frac"] for s in segs)
    com = tuple(
        sum(s["mass_frac"] * s["pos"][i] for s in segs) / total_mass_frac
        for i in range(3)
    )
    return com, segs


REQUIRED_KEYS = (
    "torso", "l_shoulder", "r_shoulder", "l_elbow", "r_elbow", "l_wrist", "r_wrist",
    "l_hip", "r_hip", "l_knee", "r_knee", "l_ankle", "r_ankle",
)


def com_series(tracks_by_marker, gender, max_dt_ns, weight_kg=None):
    """tracks_by_marker: dict marker_name -> [(t_ns, (x,y,z))] (ver
    bag_io.marker_track()), para los 13 keypoints (torso no se usa en el
    modelo de segmentos en si -- de Leva no tiene un segmento "torso
    interior" propio, el tronco va cadera-hombro -- pero se exige igualmente
    en REQUIRED_KEYS por si un futuro modelo lo quiere usar como referencia
    adicional del tronco).

    Devuelve [(t_ns, com_xyz)], emparejando cada muestra de l_hip (arbitrario,
    cualquier marcador con buena cobertura temporal serviria de "reloj")
    con la mas cercana (dentro de max_dt_ns) del resto de marcadores."""
    from . import bag_io

    missing = [k for k in REQUIRED_KEYS if k not in tracks_by_marker]
    if missing:
        raise ValueError(f"faltan marcadores requeridos: {missing}")

    queries = {k: bag_io.indexed_nearest(v) for k, v in tracks_by_marker.items() if k != "l_hip"}

    out = []
    for t, l_hip_pos in tracks_by_marker["l_hip"]:
        markers = {"l_hip": l_hip_pos}
        ok = True
        for k, q in queries.items():
            found = q(t, max_dt_ns)
            if found is None:
                ok = False
                break
            markers[k] = found[1]
        if not ok:
            continue
        com, _ = estimate_com_at_frame(markers, gender, weight_kg)
        out.append((t, com))
    return out
