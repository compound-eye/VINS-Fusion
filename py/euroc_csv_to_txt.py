import os

# Replace output_dir with your output_dir
output_dir = "/path/to/vins-fusion/output_dir"
filename = "vio_loop"
csv_path = os.path.join(output_dir, f"{filename}.csv")
with open(csv_path, "r") as f:
    lines = f.readlines()

tum_lines = []
for line in lines:
    line = line.strip().rstrip(",")  # remove trailing comma
    vals = line.split(",")
    t = float(vals[0]) / 1e9  # nanoseconds -> seconds
    px, py, pz = vals[1], vals[2], vals[3]
    qx, qy, qz, qw = vals[4], vals[5], vals[6], vals[7]
    # TUM format: t tx ty tz qx qy qz qw
    tum_lines.append(f"{t:.9f} {px} {py} {pz} {qx} {qy} {qz} {qw}")

txt_path = os.path.join(output_dir, f"{filename}_tum.txt")
with open(txt_path, "w") as f:
    f.write("\n".join(tum_lines))
