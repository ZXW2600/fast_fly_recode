import csv
import matplotlib.pyplot as plt
from scipy.integrate import solve_ivp, RK45

import argparse
import sys, os

BASEPATH = os.path.abspath(__file__).split("fast_fly_dt", 1)[0] + "fast_fly_dt"
sys.path += [BASEPATH, BASEPATH + "/script"]

parser = argparse.ArgumentParser(
    prog="无人机状态推演工具", description="重新推算给定CSV的状态图线", epilog=""
)
parser.add_argument("filename")  # positional argument
parser.add_argument("output_filename")  # positional argument

args = parser.parse_args()


filename = args.filename.split("fast_fly_dt", 1)[-1].split(".", 1)[0]

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

# csv file header:
# ['t', 'dt',2
# 'p_x', 'p_y', 'p_z',3
# 'v_x', 'v_y', 'v_z',3
# 'q_w', 'q_x', 'q_y', 'q_z',4
# 'w_x', 'w_y', 'w_z',3
# 'u_1', 'u_2', 'u_3', 'u_4']4
import scipy
import numpy as np

# old_data=np.zeros((len(data["t"],19)),np.float64)
old_data = np.array(
    [
        data["t"],
        data["p_x"],
        data["p_y"],
        data["p_z"],
        data["v_x"],
        data["v_y"],
        data["v_z"],
        data["q_w"],
        data["q_x"],
        data["q_y"],
        data["q_z"],
        data["w_x"],
        data["w_y"],
        data["w_z"],
        data["u_1"],
        data["u_2"],
        data["u_3"],
        data["u_4"],
    ]
).T

from quadrotor import QuadrotorModel
import csv

quad = QuadrotorModel(BASEPATH + "/config/quad_real.yaml")
f_dy = quad.dynamics()

u1_list = []
u2_list = []
u3_list = []
u4_list = []


def get_u(t):
    u = [0, 0, 0, 0]
    for i in range(old_data.shape[0]):
        old_t = old_data[i][0]
        if old_t < t:
            u = old_data[i][14:]
    u1_list.append(u[0])
    u2_list.append(u[1])
    u3_list.append(u[2])
    u4_list.append(u[3])


def exponential_decay(t, y):
    u = [0, 0, 0, 0]
    for i in range(old_data.shape[0]):
        old_t = old_data[i][0]
        if old_t < t:
            u = old_data[i][14:]

    return np.array(f_dy(y, u)).flatten()


yr = solve_ivp(
    fun=exponential_decay,
    t_span=[0, old_data[-1][0]],
    y0=np.array(old_data[0][1:14]).flatten(),
    method="RK23",
    rtol=1e-6,
)
for t_i in yr.t:
    get_u(t_i)

new_data = {
    "t": yr.t,
    "p_x": yr.y[0],
    "p_y": yr.y[1],
    "p_z": yr.y[2],
    "v_x": yr.y[3],
    "v_y": yr.y[4],
    "v_z": yr.y[5],
    "q_w": yr.y[6],
    "q_x": yr.y[7],
    "q_y": yr.y[8],
    "q_z": yr.y[9],
    "w_x": yr.y[10],
    "w_y": yr.y[11],
    "w_z": yr.y[12],
    "u_1": u1_list,
    "u_2": u2_list,
    "u_3": u3_list,
    "u_4": u4_list,
}

diff_data = {
    "p_x": [],
    "p_y": [],
    "p_z": [],
    "v_x": [],
    "v_y": [],
    "v_z": [],
    "q_w": [],
    "q_x": [],
    "q_y": [],
    "q_z": [],
    "w_x": [],
    "w_y": [],
    "w_z": [],
}
for i in range(len(old_data)):
    t = old_data[i][0]
    state = old_data[i][1:14]

    dt = exponential_decay(t, state)
    diff_data["p_x"].append(dt[0])
    diff_data["p_y"].append(dt[1])
    diff_data["p_z"].append(dt[2])
    diff_data["v_x"].append(dt[3])
    diff_data["v_y"].append(dt[4])
    diff_data["v_z"].append(dt[5])
    diff_data["q_w"].append(dt[6])
    diff_data["q_x"].append(dt[7])
    diff_data["q_y"].append(dt[8])
    diff_data["q_z"].append(dt[9])
    diff_data["w_x"].append(dt[10])
    diff_data["w_y"].append(dt[11])
    diff_data["w_z"].append(dt[12])

with open("diff.csv", "w") as f:
    csv_writer = csv.writer(f)
    csv_writer.writerow(
        [
            "t",
            "p_x",
            "p_y",
            "p_z",
            "v_x",
            "v_y",
            "v_z",
            "q_w",
            "q_x",
            "q_y",
            "q_z",
            "w_x",
            "w_y",
            "w_z",
        ]
    )
    for i in range(len(old_data)):
        csv_writer.writerow(
            [old_data[i][0]]
            + [
                diff_data["p_x"][i],
                diff_data["p_y"][i],
                diff_data["p_z"][i],
                diff_data["v_x"][i],
                diff_data["v_y"][i],
                diff_data["v_z"][i],
                diff_data["q_w"][i],
                diff_data["q_x"][i],
                diff_data["q_y"][i],
                diff_data["q_z"][i],
                diff_data["w_x"][i],
                diff_data["w_y"][i],
                diff_data["w_z"][i],
            ]
        )

# Specify the file path to save the CSV file
csv_file_path = args.output_filename

# Extract the keys and values from the new_data dictionary
keys = new_data.keys()
values = new_data.values()

# Write the data to the CSV file
with open(csv_file_path, "w", newline="") as csvfile:
    writer = csv.writer(csvfile)
    writer.writerow(keys)  # Write the header row
    writer.writerows(zip(*values))  # Write the data rows

new_dt = (yr.t[1:] - yr.t[0:-1]).tolist()
new_dt += [new_dt[-1]]
new_data["dt"] = np.array(new_dt)


def scatter_pt(t, p, dt, ax, fig):
    ax_dt = ax.twinx()  # instantiate a second axes that shares the same x-axis
    ax_dt.plot(t, dt, "--", color="red")

    scatter_pxt = ax.scatter(x=t, y=p, c=dt)
    ax.plot(t, p)

    fig.colorbar(scatter_pxt, label="dt", ax=ax)


def plot_tu(data, name):
    fig_tu = plt.figure()
    fig_tu.canvas.manager.set_window_title(f"t-u :{filename} {name}")
    ax_u1, ax_u2, ax_u3, ax_u4 = fig_tu.subplots(4, 1)
    scatter_pt(data["t"], data["u_1"], data["dt"], ax_u1, fig_tu)
    scatter_pt(data["t"], data["u_2"], data["dt"], ax_u2, fig_tu)
    scatter_pt(data["t"], data["u_3"], data["dt"], ax_u3, fig_tu)
    scatter_pt(data["t"], data["u_4"], data["dt"], ax_u4, fig_tu)


def plot_tp(data, name):
    fig_tp = plt.figure()
    fig_tp.canvas.manager.set_window_title(f"t-p :{filename} {name}")

    ax_px, ax_py, ax_pz = fig_tp.subplots(3, 1)

    scatter_pt(data["t"], data["p_x"], data["dt"], ax_px, fig_tp)
    scatter_pt(data["t"], data["p_y"], data["dt"], ax_py, fig_tp)
    scatter_pt(data["t"], data["p_z"], data["dt"], ax_pz, fig_tp)


plot_tp(data, "old")
plot_tp(new_data, "new")


plt.show()
