import csv
import matplotlib.pyplot as plt

import argparse


parser = argparse.ArgumentParser(prog="无人机状态绘图工具", description="绘制给定CSV的图线", epilog="")
parser.add_argument("filename")  # positional argument
parser.add_argument("-t", "--type", nargs="*", help="可以为: [tp tu]")

args = parser.parse_args()
if args.type == None:
    args.type = ["tp", "tu"]

filename = args.filename.split("dt_opt", 1)[-1].split(".", 1)[0]

print(f"filename: {filename}", end=" ->")
data = {}
with open(args.filename, newline="") as csvfile:
    spamreader = csv.reader(csvfile, delimiter=",", quotechar="|")
    index = -1
    header = []
    for row in spamreader:
        if index == -1:
            header = row
            for var in header:
                data[var] = []
            print(f"csv file header:{header}")
        else:
            for i in range(len(header)):
                data[header[i]] += [float(row[i])]
        index += 1
    # print(data)
# if dt data not exist, make on with dt=0.1
if "dt" not in header:
    data["dt"] = [0.1] * len(data["t"])

# csv file header:
# ['t', 'dt',
# 'p_x', 'p_y', 'p_z',
# 'v_x', 'v_y', 'v_z',
# 'q_w', 'q_x', 'q_y', 'q_z',
# 'w_x', 'w_y', 'w_z',
# 'u_1', 'u_2', 'u_3', 'u_4']


def scatter_pt(t, p, dt, ax, fig):
    ax_dt = ax.twinx()  # instantiate a second axes that shares the same x-axis
    ax_dt.plot(t, dt, "--", color="red")

    scatter_pxt = ax.scatter(x=t, y=p, c=dt)
    ax.plot(t, p)

    fig.colorbar(scatter_pxt, label="dt", ax=ax)


if "tu" in args.type and "u_1" in header:
    fig_tu = plt.figure()
    fig_tu.canvas.manager.set_window_title(f"t-u :{filename}")
    ax_u1, ax_u2, ax_u3, ax_u4 = fig_tu.subplots(4, 1)
    scatter_pt(data["t"], data["u_1"], data["dt"], ax_u1, fig_tu)
    scatter_pt(data["t"], data["u_2"], data["dt"], ax_u2, fig_tu)
    scatter_pt(data["t"], data["u_3"], data["dt"], ax_u3, fig_tu)
    scatter_pt(data["t"], data["u_4"], data["dt"], ax_u4, fig_tu)

if "tp" in args.type  and "p_x" in header:
    fig_tp = plt.figure()
    fig_tp.canvas.manager.set_window_title(f"t-p :{filename}")

    ax_px, ax_py, ax_pz = fig_tp.subplots(3, 1)

    scatter_pt(data["t"], data["p_x"], data["dt"], ax_px, fig_tp)
    scatter_pt(data["t"], data["p_y"], data["dt"], ax_py, fig_tp)
    scatter_pt(data["t"], data["p_z"], data["dt"], ax_pz, fig_tp)

plt.show()
