import subprocess
import re
import time

print("Recording ground truth... Press Ctrl+C after the drone lands.")
with open('gt_log.csv', 'w') as f:
    f.write('t,x,y,z\n')
    
    proc = subprocess.Popen(
        ['gz', 'topic', '-e', '-t', '/world/precision_landing/dynamic_pose/info'], 
        stdout=subprocess.PIPE, 
        text=True
    )
    
    buf = ""
    try:
        for line in proc.stdout:
            buf += line
            # Match the drone pose block
            m = re.search(
                r'sec:\s*(\d+).*?name: "x500_pl_0".*?position\s*\{(.*?)\}', 
                buf, 
                re.DOTALL
            )
            if m:
                t = m.group(1)
                pos_block = m.group(2)
                
                # Protobuf sometimes omits 0.0 values, so check safely
                x_m = re.search(r'x:\s*([-0-9.]+)', pos_block)
                y_m = re.search(r'y:\s*([-0-9.]+)', pos_block)
                z_m = re.search(r'z:\s*([-0-9.]+)', pos_block)
                
                x = float(x_m.group(1)) if x_m else 0.0
                y = float(y_m.group(1)) if y_m else 0.0
                z = float(z_m.group(1)) if z_m else 0.0
                
                f.write(f"{t},{x},{y},{z}\n")
                f.flush()
                buf = ""  # Clear buffer to avoid memory bloat
                time.sleep(0.05) # ~20 Hz is plenty for landing eval
    except KeyboardInterrupt:
        print("\nStopped logging.")
    finally:
        proc.terminate()
