import csv
import statistics as st

rows = list(csv.DictReader(open("results.csv")))
print(f"{'config':<8} | {'n':>2} | {'ok':>3} | {'mean':>7} | {'median':>7} | {'p90':>7} | {'max':>7} (cm radial)")
print("-" * 70)
for perc in ("aruco", "yolo"):
    sel = [r for r in rows if r["perception"] == perc]
    if not sel:
        continue
    ok = [r for r in sel if r["success"] == "True"]
    rad = sorted(float(r["err_radial_cm"]) for r in ok)
    if not rad:
        print(f"{perc:<8} | {len(sel):>2} | {0:>3} |   no successful trials")
        continue
    p90 = rad[min(len(rad) - 1, int(0.9 * (len(rad) - 1) + 0.5))]
    print(f"{perc:<8} | {len(sel):>2} | {len(ok):>3} | "
          f"{st.mean(rad):>7.2f} | {st.median(rad):>7.2f} | {p90:>7.2f} | {rad[-1]:>7.2f}")
