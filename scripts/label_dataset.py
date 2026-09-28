"""
Label the harvested dataset using ArUco detections.

For each image:
  - Run ArUco offline
  - Write a YOLO-format .txt label (class, cx, cy, w, h normalized)
  - Save an overlay image for visual sanity check

The ML model will learn to predict what ArUco predicts, then beat it
on corrupted/edge-case inputs at inference time.
"""
import os
import sys

import cv2
import numpy as np

IMG_DIR = "dataset/images"
LABELS_DIR = "dataset/labels"
OVERLAY_DIR = "dataset/overlay"

os.makedirs(LABELS_DIR, exist_ok=True)
os.makedirs(OVERLAY_DIR, exist_ok=True)

aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
params = cv2.aruco.DetectorParameters()
detector = cv2.aruco.ArucoDetector(aruco_dict, params)

files = sorted(f for f in os.listdir(IMG_DIR) if f.endswith(".png"))
if not files:
    print("no images found")
    sys.exit(1)

n_labeled = 0
n_total = len(files)

for fn in files:
    path = os.path.join(IMG_DIR, fn)
    img = cv2.imread(path)
    if img is None:
        continue
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    corners, ids, _ = detector.detectMarkers(gray)
    H, W = img.shape[:2]

    label_path = os.path.join(LABELS_DIR, fn.replace(".png", ".txt"))
    overlay = img.copy()

    if ids is not None:
        # ArUco detected the tag — extract its bbox
        # Use the first (only) detection
        corner = corners[0].reshape(4, 2)
        xmin, ymin = corner.min(axis=0)
        xmax, ymax = corner.max(axis=0)
        cx = (xmin + xmax) / 2.0
        cy = (ymin + ymax) / 2.0
        bw = xmax - xmin
        bh = ymax - ymin

        # YOLO format: <class> <cx/W> <cy/H> <bw/W> <bh/H>
        with open(label_path, "w") as f:
            f.write(
                f"0 {cx / W:.6f} {cy / H:.6f} {bw / W:.6f} {bh / H:.6f}\n"
            )

        # Draw overlay for sanity
        cv2.rectangle(overlay,
                      (int(xmin), int(ymin)),
                      (int(xmax), int(ymax)),
                      (0, 255, 0), 2)
        cv2.putText(overlay, f"id={int(ids[0][0])}",
                    (int(xmin), int(ymin) - 6),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
        n_labeled += 1
    else:
        # No detection — write empty label (negative example)
        open(label_path, "w").close()

    cv2.imwrite(os.path.join(OVERLAY_DIR, fn), overlay)

print(f"Labeled {n_labeled}/{n_total} frames ({100*n_labeled/n_total:.1f}% had ArUco)")
print(f"Overlays in {OVERLAY_DIR}/ — inspect a few to confirm labels are correct.")
