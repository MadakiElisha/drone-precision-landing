import csv
import numpy as np

rows = list(csv.DictReader(open('replay_data.csv')))
dets = []
for r in rows:
    if r['type'] == 'det':
        dets.append((int(r['t_ns']), float(r['x']), float(r['y']), float(r['z'])))

def run_filter(dets, q, gate):
    x = None; P = None; pred_time = dets[0][0]
    rejected = 0; accepted = 0
    r_rel = 0.02; rz_rel = 0.06
    
    for t, ox, oy, oz in dets:
        z = np.array([ox, oy, oz])
        dt = (t - pred_time) * 1e-9
        if dt > 0 and x is not None:
            dt = min(dt, 0.5)
            F = np.eye(6); F[0,3]=F[1,4]=F[2,5]=dt
            Q = np.diag([(q*dt*dt/2)**2]*3 + [(q*dt)**2]*3)
            x = F @ x; P = F @ P @ F.T + Q
            pred_time = t
            
        if x is None:
            x = np.array([ox,oy,oz,0,0,0])
            P = np.diag([0.01]*3 + [0.25]*3)
            accepted += 1; continue
            
        oz_est = max(abs(x[2]), 0.2)
        R = np.diag([(r_rel*oz_est)**2, (r_rel*oz_est)**2, (rz_rel*oz_est)**2])
        H = np.hstack((np.eye(3), np.zeros((3,3))))
        S = H @ P @ H.T + R
        innov = z - H @ x
        if float(innov @ np.linalg.solve(S, innov)) > gate:
            rejected += 1; continue
            
        K = P @ H.T @ np.linalg.inv(S)
        x = x + K @ innov
        I_KH = np.eye(6) - K @ H
        P = I_KH @ P @ I_KH.T + K @ R @ K.T
        P = (P + P.T)/2.0
        accepted += 1
        
    return accepted, rejected

print(f"Loaded {len(dets)} raw detections.")
print("\n Q    | Gate | Accepted | Rejected | Reject %")
print("-" * 45)
for q in [0.1, 1.5, 5.0, 20.0, 50.0]:
    for gate in [16.0, 50.0, 100.0]:
        acc, rej = run_filter(dets, q, gate)
        total = acc + rej
        pct = 100*rej/total if total else 0
        print(f" {q:<4.1f} | {gate:<4.0f} | {acc:<8} | {rej:<8} | {pct:.1f}%")
