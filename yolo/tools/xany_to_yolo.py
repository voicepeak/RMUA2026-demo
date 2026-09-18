#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""X-AnyLabeling 原生 JSON -> YOLO detection 数据集。

用法:
  python3 xany_to_yolo.py --src <带 *_L.json/_R.json 的目录> --out <数据集目录> [--val-seqs 15,16,17,18]
生成:
  <out>/images/{train,val}/*.png
  <out>/labels/{train,val}/*.txt   每行: 0 cx cy w h (归一化)
  <out>/data.yaml
"""

import argparse
import glob
import json
import os
import shutil

import cv2

CLASSES = {"gate": 0}


def seq_of(name):
    # 000090_L -> 90
    try:
        return int(os.path.basename(name).split("_")[0])
    except ValueError:
        return 0


def load_json(path):
    with open(path) as f:
        return json.load(f)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--src", required=True)
    ap.add_argument("--out", required=True)
    ap.add_argument("--val-seqs", default="", help="逗号分隔的序列号, 用作 val")
    args = ap.parse_args()

    jsons = sorted(glob.glob(os.path.join(args.src, "*_L.json")) +
                   glob.glob(os.path.join(args.src, "*_R.json")))
    if not jsons:
        print("no json found in", args.src)
        return
    val_seqs = set(int(x) for x in args.val_seqs.split(",") if x.strip() != "")

    for sub in ("images/train", "images/val", "labels/train", "labels/val"):
        os.makedirs(os.path.join(args.out, sub), exist_ok=True)

    n_img = n_box = 0
    for jp in jsons:
        base = os.path.splitext(os.path.basename(jp))[0]
        img_src = os.path.join(args.src, base + ".png")
        if not os.path.exists(img_src):
            print("missing image for", jp)
            continue
        img = cv2.imread(img_src)
        H, W = img.shape[:2]
        data = load_json(jp)
        lines = []
        for sh in data.get("shapes", []):
            label = sh.get("label", "")
            if label not in CLASSES:
                continue
            pts = sh.get("points", [])
            if len(pts) < 2:
                continue
            xs = [p[0] for p in pts]
            ys = [p[1] for p in pts]
            x0, x1 = max(0.0, min(xs)), min(W - 1.0, max(xs))
            y0, y1 = max(0.0, min(ys)), min(H - 1.0, max(ys))
            if x1 - x0 < 2 or y1 - y0 < 2:
                continue
            cx = ((x0 + x1) / 2) / W
            cy = ((y0 + y1) / 2) / H
            w = (x1 - x0) / W
            h = (y1 - y0) / H
            lines.append("%d %.6f %.6f %.6f %.6f" % (CLASSES[label], cx, cy, w, h))
            n_box += 1
        split = "val" if seq_of(base) in val_seqs else "train"
        shutil.copy(img_src, os.path.join(args.out, "images", split, base + ".png"))
        with open(os.path.join(args.out, "labels", split, base + ".txt"), "w") as f:
            f.write("\n".join(lines) + ("\n" if lines else ""))
        n_img += 1

    with open(os.path.join(args.out, "data.yaml"), "w") as f:
        f.write("path: %s\n" % os.path.abspath(args.out))
        f.write("train: images/train\nval: images/val\n")
        f.write("names:\n  0: gate\n")
    print("images=%d boxes=%d  val_seqs=%s -> %s" % (n_img, n_box, sorted(val_seqs), args.out))


if __name__ == "__main__":
    main()
