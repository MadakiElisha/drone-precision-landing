import csv
import math

try:
    with open('gt_log.csv', 'r') as f:
        reader = list(csv.DictReader(f))
        
    if not reader:
        print("No data in gt_log.csv!")
        exit()

    last = reader[-1]
    x, y = float(last['x']), float(last['y'])

    pad_x, pad_y = 4.0, 0.0
    err_x_cm = (x - pad_x) * 100
    err_y_cm = (y - pad_y) * 100
    err_total_cm = math.hypot(err_x_cm, err_y_cm)

    print(f"=== BASELINE TOUCHDOWN ERROR ===")
    print(f"Final Drone Pos: ({x:.3f}, {y:.3f})")
    print(f"Pad Pos:         ({pad_x:.3f}, {pad_y:.3f})")
    print(f"Error:           X={err_x_cm:.2f} cm, Y={err_y_cm:.2f} cm")
    print(f"Total Radial Error: {err_total_cm:.2f} cm")
    print(f"================================")
except FileNotFoundError:
    print("gt_log.csv not found. Run log_gt.py first!")
