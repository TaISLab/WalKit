#!/usr/bin/env python3
"""Etapa 2: WORK/stage2/<bag> = /scan_odom (etapa 1) + /bwt901cl/imu y /bwt901cl/imu_filtered (bag original),
ordenados por tiempo, para que harness.py los reproduzca juntos con un unico /clock.
Uso: python3 build_stage2.py"""
import shutil
from multiprocessing import Pool

import rosbag2_py
import yaml

import tlib

W = tlib.WORK


def raw_msgs(bag, topics):
    meta = yaml.safe_load(open(f"{bag}/metadata.yaml"))["rosbag2_bagfile_information"]
    r = rosbag2_py.SequentialReader()
    r.open(rosbag2_py.StorageOptions(uri=str(bag), storage_id=meta.get("storage_identifier", "sqlite3")),
           rosbag2_py.ConverterOptions("", ""))
    types = {t.name: t.type for t in r.get_all_topics_and_types()}
    r.set_filter(rosbag2_py.StorageFilter(topics=[t for t in topics if t in types]))
    out = []
    while r.has_next():
        t, d, ts = r.read_next()
        out.append((ts, t, d))
    return out, types


def build(bag):
    try:
        dst = W / "stage2" / bag
        if (dst / "metadata.yaml").exists():
            return bag, "skip"
        if not (W / "stage1" / bag / "metadata.yaml").exists():
            return bag, "SIN ETAPA 1"
        if dst.exists():
            shutil.rmtree(dst)
        a, ta = raw_msgs(W / "stage1" / bag, ["/scan_odom"])
        b, tb = raw_msgs(tlib.BAGS_DIR / bag, ["/bwt901cl/imu", "/bwt901cl/imu_filtered"])
        types = {**ta, **tb}
        w = rosbag2_py.SequentialWriter()
        w.open(rosbag2_py.StorageOptions(uri=str(dst), storage_id="sqlite3"), rosbag2_py.ConverterOptions("", ""))
        for name in ("/scan_odom", "/bwt901cl/imu", "/bwt901cl/imu_filtered"):
            if name in types:
                w.create_topic(rosbag2_py.TopicMetadata(id=0, name=name, type=types[name], serialization_format="cdr"))
        for ts, t, d in sorted(a + b, key=lambda m: m[0]):
            w.write(t, d, ts)
        del w
        return bag, f"ok {len(a)} scan_odom {len(b)} imu"
    except Exception as e:
        return bag, "ERROR " + repr(e)[:120]


if __name__ == "__main__":
    (W / "stage2").mkdir(parents=True, exist_ok=True)
    with Pool(8) as p:
        for r in p.imap_unordered(build, tlib.ALL_BAGS):
            print(r, flush=True)
