import os
import time
import cv2
import numpy as np
from ultralytics import YOLO

IMG_DIR = "dataset/val/images"

# Setup ArUco
aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
params = cv2.aruco.DetectorParameters()
detector = cv2.aruco.ArucoDetector(aruco_dict, params)

# Setup YOLO
yolo = YOLO("runs/detect/runs/pad_v1/weights/best.pt")

# Warmup YOLO so the first inference isn't penalized by JIT/compilation
dummy = np.zeros((416, 416, 3), dtype=np.uint8)
yolo(dummy, verbose=False)

corruptions = {
    'clean':   lambda x: x,
    'blur':    lambda x: cv2.GaussianBlur(x, (21, 21), 0),
    'noise':   lambda x: np.clip(x + np.random.normal(0, 40, x.shape), 0, 255).astype(np.uint8),
    'dark':    lambda x: (x.astype(np.float32) * 0.2).astype(np.uint8),
    'occlude': lambda x: np.concatenate([x[:x.shape[0]//2], np.zeros_like(x[x.shape[0]//2:])], axis=0)
}

files = sorted([f for f in os.listdir(IMG_DIR) if f.endswith('.png')])
n = len(files)

print(f"{'Corruption':<10} | {'ArUco Det':>10} | {'ArUco ms':>10} | {'YOLO Det':>10} | {'YOLO ms':>10}")
print("-" * 65)

for name, corrupt_fn in corruptions.items():
    aruco_hits, aruco_time = 0, 0
    yolo_hits, yolo_time = 0, 0
    
    for fn in files:
        img = cv2.imread(os.path.join(IMG_DIR, fn))
        if img is None: continue
        img_c = corrupt_fn(img)
        gray = cv2.cvtColor(img_c, cv2.COLOR_BGR2GRAY)
        
        # --- ArUco ---
        t0 = time.perf_counter()
        corners, ids, _ = detector.detectMarkers(gray)
        aruco_time += time.perf_counter() - t0
        if ids is not None: aruco_hits += 1
        
        # --- YOLO ---
        t0 = time.perf_counter()
        res = yolo(img_c, verbose=False)
        yolo_time += time.perf_counter() - t0
        if res[0].boxes.shape[0] > 0: yolo_hits += 1
        
    print(f"{name:<10} | {aruco_hits:>4}/{n:<5} | {aruco_time/n*1000:>8.1f}ms | {yolo_hits:>4}/{n:<5} | {yolo_time/n*1000:>8.1f}ms")
