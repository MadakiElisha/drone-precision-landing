import csv
import numpy as np

rows = list(csv.DictReader(open('replay_data.csv')))
dets = []
for r in rows:
    if r['type'] == 'det':
        dets.append((float(r['x']), float(r['y']), float(r['z'])))

dets = np.array(dets)

def calc_jitter(traj):
    # High-frequency jitter is the variance of the differences (acceleration proxy)
    diffs = np.diff(traj, axis=0)
    return np.mean(np.var(diffs, axis=0))

raw_jitter = calc_jitter(dets)
print(f"Raw YOLO jitter metric: {raw_jitter:.6f}")
print("\n Alpha | Jitter Reduction %")
print("-" * 30)

for alpha in [0.01, 0.05, 0.1, 0.2, 0.3, 0.5, 0.8]:
    smoothed = np.zeros_like(dets)
    smoothed[0] = dets[0]
    for i in range(1, len(dets)):
        smoothed[i] = alpha * dets[i] + (1 - alpha) * smoothed[i-1]
    
    sm_jitter = calc_jitter(smoothed)
    reduction = 100 * (1 - sm_jitter / raw_jitter)
    print(f" {alpha:<4.2f} | {reduction:>5.1f}%")
