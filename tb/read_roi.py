#!/usr/bin/env python3
"""Đọc ROI do u01_model ghi ra (+U01_DUMP) để đưa vào mô hình phần mềm của U01.
   python3 read_roi.py roi_f0_id3_64x48.hex  [--png out.png]
   Trả về mảng numpy (H, W, 3) thứ tự byte R, G, B (HWC) đúng như U01 thật nhận."""
import re, sys, numpy as np

def load_roi(path):
    w, h = map(int, re.search(r"_(\d+)x(\d+)\.hex$", path).groups())
    data = np.array([int(l, 16) for l in open(path) if l.strip()], dtype=np.uint8)
    assert data.size == w * h * 3, f"{path}: {data.size} B, mong đợi {w*h*3}"
    return data.reshape(h, w, 3)

if __name__ == "__main__":
    a = load_roi(sys.argv[1])
    print(sys.argv[1], "shape", a.shape, "min/max", a.min(), a.max())
    if "--png" in sys.argv:
        from PIL import Image
        Image.fromarray(a).resize((a.shape[1] * 4, a.shape[0] * 4), Image.NEAREST).save(sys.argv[sys.argv.index("--png") + 1])
