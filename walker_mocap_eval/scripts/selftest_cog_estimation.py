#!/usr/bin/env python3
"""Auto-test con datos sinteticos (sin bags, sin ROS) para cog_estimation.py:
comprueba que las fracciones de masa de De Leva suman 1.0 (con y sin
sesgo), y que, para una postura sencilla y simetrica con geometria
conocida, el CoM calculado coincide con el que sale de hacerlo a mano.

Uso:
    python3 selftest_cog_estimation.py
"""
import math
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from mocap_eval import cog_estimation as cog  # noqa: E402


def approx(a, b, tol=1e-6):
    return math.isfinite(a) and math.isfinite(b) and abs(a - b) <= tol


def test_mass_fractions_sum_to_one():
    for table in (cog.DE_LEVA_MALE, cog.DE_LEVA_FEMALE):
        total = table["trunk"][0] + table["head"][0] + 2 * (
            table["upper_arm"][0] + table["forearm"][0] + table["hand"][0] +
            table["thigh"][0] + table["shank"][0] + table["foot"][0])
        assert approx(total, 1.0, tol=1e-3), total
    print("test_mass_fractions_sum_to_one: OK")


def _standing_pose(z_hip=0.9, z_shoulder=1.4, z_elbow=1.1, z_wrist=0.8, z_knee=0.45, z_ankle=0.0, y_half=0.15):
    """Postura simetrica, de pie, brazos pegados al cuerpo, sin flexion
    lateral -- todo en el plano x=0, simetrico en y=+-y_half."""
    m = {}
    for side, sign in (("l", +1), ("r", -1)):
        y = sign * y_half
        m[f"{side}_shoulder"] = (0.0, y, z_shoulder)
        m[f"{side}_elbow"] = (0.0, y, z_elbow)
        m[f"{side}_wrist"] = (0.0, y, z_wrist)
        m[f"{side}_hip"] = (0.0, y * 0.7, z_hip)
        m[f"{side}_knee"] = (0.0, y * 0.7, z_knee)
        m[f"{side}_ankle"] = (0.0, y * 0.7, z_ankle)
    m["torso"] = (0.0, 0.0, (z_hip + z_shoulder) / 2.0)
    return m


def test_standing_pose_com_matches_hand_calc():
    markers = _standing_pose()
    gender = "m"
    table = cog.DE_LEVA_MALE

    com, segs = cog.estimate_com_at_frame(markers, gender)

    # Simetria: x=0 e y=0 exactos (postura perfectamente simetrica).
    assert approx(com[0], 0.0), com
    assert approx(com[1], 0.0, tol=1e-9), com

    # Calculo a mano del z del CoM, segmento a segmento (mismas formulas
    # que estimate_com_at_frame, pero explicitas aqui para no depender del
    # propio codigo bajo prueba).
    z_hip, z_shoulder, z_elbow, z_wrist, z_knee, z_ankle = 0.9, 1.4, 1.1, 0.8, 0.45, 0.0
    hip_mid_z = z_hip
    shoulder_mid_z = z_shoulder

    trunk_frac, trunk_com = table["trunk"]
    trunk_z = hip_mid_z + trunk_com * (shoulder_mid_z - hip_mid_z)

    head_frac, _ = table["head"]
    head_z = shoulder_mid_z

    ua_frac, ua_com = table["upper_arm"]
    ua_z = z_shoulder + ua_com * (z_elbow - z_shoulder)

    fa_frac, fa_com = table["forearm"]
    fa_z = z_elbow + fa_com * (z_wrist - z_elbow)

    hand_frac, _ = table["hand"]
    hand_z = z_wrist

    th_frac, th_com = table["thigh"]
    th_z = z_hip + th_com * (z_knee - z_hip)

    sh_frac, sh_com = table["shank"]
    sh_z = z_knee + sh_com * (z_ankle - z_knee)

    foot_frac, _ = table["foot"]
    foot_z = z_ankle

    num = (trunk_frac * trunk_z + head_frac * head_z +
           2 * (ua_frac * ua_z + fa_frac * fa_z + hand_frac * hand_z +
                th_frac * th_z + sh_frac * sh_z + foot_frac * foot_z))
    den = (trunk_frac + head_frac +
           2 * (ua_frac + fa_frac + hand_frac + th_frac + sh_frac + foot_frac))
    expected_z = num / den

    assert approx(com[2], expected_z, tol=1e-9), (com[2], expected_z)

    # Rango de sensatez: el CoM de un adulto de pie cae tipicamente sobre el
    # ombligo/L2-L3, entre la cadera y el hombro, mas cerca de la cadera.
    assert z_hip < com[2] < z_shoulder, com

    print("test_standing_pose_com_matches_hand_calc: OK (com_z=%.4f, esperado=%.4f)" % (com[2], expected_z))


def test_forward_lean_shifts_com_forward():
    upright = _standing_pose()
    com_upright, _ = cog.estimate_com_at_frame(upright, "f")

    leaned = _standing_pose()
    # Inclina el tronco hacia +x sin mover caderas/piernas (aproxima una
    # flexion de cadera hacia delante).
    for k in ("l_shoulder", "r_shoulder", "l_elbow", "r_elbow", "l_wrist", "r_wrist"):
        x, y, z = leaned[k]
        leaned[k] = (x + 0.3, y, z)
    com_leaned, _ = cog.estimate_com_at_frame(leaned, "f")

    assert com_leaned[0] > com_upright[0], (com_upright, com_leaned)
    print("test_forward_lean_shifts_com_forward: OK (dx=%.4f)" % (com_leaned[0] - com_upright[0]))


def test_unknown_gender_raises():
    try:
        cog.estimate_com_at_frame(_standing_pose(), "x")
    except ValueError:
        print("test_unknown_gender_raises: OK")
        return
    raise AssertionError("esperaba ValueError para sexo desconocido")


if __name__ == "__main__":
    test_mass_fractions_sum_to_one()
    test_standing_pose_com_matches_hand_calc()
    test_forward_lean_shifts_com_forward()
    test_unknown_gender_raises()
    print("\nTodos los selftests OK")
