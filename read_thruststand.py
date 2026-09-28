import matplotlib
matplotlib.use("TkAgg")   # ep dung backend co GUI de ve real-time on dinh

import serial
import serial.tools.list_ports
import time
import csv
import numpy as np
import matplotlib.pyplot as plt
from scipy.optimize import curve_fit

BAUD = 115200

SWEEP_CSV = "sweep_data.csv"
STEP_CSV  = "step_data.csv"
TF_TXT    = "motor_transfer_function.txt"

# ---------------- TU DO TIM COM PORT ----------------

def find_esp32_port():
    keywords = ["CP210", "CH340", "USB-SERIAL", "USB JTAG", "SLAB", "USB Serial"]

    print("Dang doi cong COM cua ESP32...")
    while True:
        ports = list(serial.tools.list_ports.comports())
        candidates = []
        for p in ports:
            desc = (p.description or "") + " " + (p.manufacturer or "")
            if any(k.lower() in desc.lower() for k in keywords):
                candidates.append(p)

        if candidates:
            chosen = candidates[0]
            print("Cac cong COM tim thay:")
            for p in ports:
                marker = " <-- chon" if p.device == chosen.device else ""
                print(f"  {p.device}: {p.description}{marker}")
            return chosen.device

        if ports:
            available = ", ".join(f"{p.device}: {p.description}" for p in ports)
            print(f"Chua thay ESP32; cac cong dang co: {available}")
        else:
            print("Chua thay cong COM nao.")
        time.sleep(1)


PORT = find_esp32_port()
print(f"Dang mo cong {PORT} ...")
ser = serial.Serial(PORT, BAUD, timeout=2)
time.sleep(2)  # doi ESP32 reset xong

plt.ion()

# ---------------- MO SAN 2 FILE CSV, GHI HEADER TRUOC ----------------
sweep_file = open(SWEEP_CSV, "w", newline="", encoding="utf-8")
sweep_writer = csv.writer(sweep_file)
sweep_writer.writerow(["time_ms", "voltage", "throttle_pct", "thrust_g", "moment_Nm"])

step_file = open(STEP_CSV, "w", newline="", encoding="utf-8")
step_writer = csv.writer(step_file)
step_writer.writerow(["repeat", "t_since_step_ms", "voltage", "thrust_g", "base_pct", "target_pct"])

# ---------------- BIEN TRANG THAI ----------------
mode = None
current_step_meta = {}
battery_voltage = None

# sweep realtime
fig_sweep = None
ax_sweep = None
sweep_scatter = None
sweep_x, sweep_y = [], []

# step realtime (4 o con)
fig_step = None
axes_step = None
step_scatter_artists = {}
step_line_artists = {}
step_axes = {}
step_data_buffers = {}
last_repeat_seen = {}
step_config_order = []


def init_sweep_figure():
    global fig_sweep, ax_sweep, sweep_scatter
    fig_sweep, ax_sweep = plt.subplots()
    ax_sweep.set_xlabel("Throttle (%)")
    ax_sweep.set_ylabel("Thrust (g)")
    ax_sweep.set_title("TEST 1 - SWEEP: Thrust vs Throttle (real-time)")
    ax_sweep.grid(True)
    sweep_scatter = ax_sweep.scatter([], [], s=8, alpha=0.5, color="tab:blue")
    fig_sweep.show()
    plt.pause(0.01)


def init_step_figure():
    global fig_step, axes_step
    fig_step, ax = plt.subplots(figsize=(7, 5))
    axes_step = [ax]
    fig_step.suptitle("TEST 2 - STEP RESPONSE (equivalent-time, real-time)")
    fig_step.show()
    plt.pause(0.01)


def get_or_create_step_axis(base, target):
    key = (base, target)
    if key in step_axes:
        return key

    idx = len(step_config_order)
    step_config_order.append(key)

    ax = axes_step[idx]
    ax.set_xlabel("t sau step (ms)")
    ax.set_ylabel("Thrust (g)")
    ax.set_title(f"{base:.0f}% -> {target:.0f}%")
    ax.grid(True)

    scatter = ax.scatter([], [], s=8, alpha=0.35, color="tab:blue", label="mau tho")
    line, = ax.plot([], [], color="tab:orange", linewidth=1.3, label="duong noi (theo repeat)")
    ax.legend(fontsize=8)

    step_axes[key] = ax
    step_scatter_artists[key] = scatter
    step_line_artists[key] = line
    step_data_buffers[key] = []
    last_repeat_seen[key] = -1

    return key


print("Dang cho du lieu tu ESP32...")

while True:
    raw = ser.readline().decode(errors="ignore").strip()
    if not raw:
        continue
    print(raw)

    if raw.startswith("# VOLTAGE="):
        try:
            battery_voltage = float(raw.split("=")[1])
        except ValueError:
            pass
        continue

    if raw.startswith("# BEGIN SWEEP"):
        mode = "SWEEP"
        if fig_sweep is None:
            init_sweep_figure()
        continue

    if raw.startswith("# END SWEEP"):
        mode = None
        sweep_file.flush()
        continue

    if raw.startswith("# BEGIN STEP"):
        mode = "STEP"
        parts = dict(p.split("=") for p in raw.replace("# BEGIN STEP ", "").split())
        current_step_meta = parts
        if fig_step is None:
            init_step_figure()
        get_or_create_step_axis(float(parts["base"]), float(parts["target"]))
        continue

    if raw.startswith("# END STEP"):
        mode = None
        step_file.flush()
        continue

    if raw.startswith("# STEP TEST"):
        continue

    if raw.startswith("# ALL TESTS DONE"):
        break

    if raw.startswith("#") or raw.startswith("time_ms") or raw.startswith("repeat"):
        continue

    fields = raw.split(",")

    # ---------------- XU LY DONG SWEEP (5 cot: time_ms,voltage,throttle_pct,thrust_g,moment_Nm) ----------------
    if mode == "SWEEP" and len(fields) == 5:
        sweep_writer.writerow(fields)

        try:
            pct_f = float(fields[2])
            thrust_f = float(fields[3])
        except ValueError:
            continue

        sweep_x.append(pct_f)
        sweep_y.append(thrust_f)
        sweep_scatter.set_offsets(np.column_stack([sweep_x, sweep_y]))
        ax_sweep.update_datalim([[pct_f, thrust_f]])
        ax_sweep.autoscale_view()
        fig_sweep.canvas.draw_idle()
        fig_sweep.canvas.flush_events()
        plt.pause(0.001)

    # ---------------- XU LY DONG STEP (4 cot: repeat,t_since_step_ms,voltage,thrust_g) ----------------
    elif mode == "STEP" and len(fields) == 4:
        base = float(current_step_meta.get("base"))
        target = float(current_step_meta.get("target"))
        key = (base, target)

        row = fields + [base, target]
        step_writer.writerow(row)

        try:
            repeat_i = int(fields[0])
            t_f = float(fields[1])
            thrust_f = float(fields[3])
        except ValueError:
            continue

        buf = step_data_buffers[key]
        buf.append((t_f, thrust_f))

        pts = np.array(buf)
        step_scatter_artists[key].set_offsets(pts)

        if repeat_i != last_repeat_seen[key]:
            last_repeat_seen[key] = repeat_i
            sorted_pts = sorted(buf, key=lambda p: p[0])
            xs = [p[0] for p in sorted_pts]
            ys = [p[1] for p in sorted_pts]
            step_line_artists[key].set_data(xs, ys)

        ax = step_axes[key]
        ax.relim()
        ax.autoscale_view()
        fig_step.canvas.draw_idle()
        fig_step.canvas.flush_events()
        plt.pause(0.001)

ser.close()
sweep_file.close()
step_file.close()

print(f"Da luu xong {SWEEP_CSV} va {STEP_CSV}")

# ---------------- TU DONG TINH HAM TRUYEN DONG CO TU DU LIEU STEP ----------------

def first_order_model(t, y0, yss, tau):
    return y0 + (yss - y0) * (1 - np.exp(-t / tau))


tf_lines = []
tf_lines.append("HAM TRUYEN DONG CO (mo hinh bac 1, uoc luong tu du lieu step)")
tf_lines.append("Dang: G(s) = K / (tau*s + 1)")
tf_lines.append("")

for key in step_config_order:
    base, target = key
    buf = step_data_buffers[key]

    if len(buf) < 4:
        line = f"Step {base:.0f}% -> {target:.0f}%: khong du diem du lieu de fit"
        print(line)
        tf_lines.append(line)
        continue

    pts = sorted(buf, key=lambda p: p[0])
    t_arr = np.array([p[0] for p in pts]) / 1000.0
    y_arr = np.array([p[1] for p in pts])

    y0_guess  = y_arr[:max(2, len(y_arr)//6)].mean()
    yss_guess = y_arr[-max(2, len(y_arr)//6):].mean()
    tau_guess = 0.05

    y_min, y_max = y_arr.min(), y_arr.max()
    y_span = max(abs(y_max - y_min), 1.0)

    lower_bounds = [y_min - y_span, y_min - y_span, 0.001]
    upper_bounds = [y_max + y_span, y_max + y_span, 2.0]

    try:
        popt, _ = curve_fit(
            first_order_model, t_arr, y_arr,
            p0=[y0_guess, yss_guess, tau_guess],
            bounds=(lower_bounds, upper_bounds),
            maxfev=20000
        )
        y0_fit, yss_fit, tau_fit = popt

        near_bound = (
            abs(tau_fit - upper_bounds[2]) < 1e-3 or
            abs(yss_fit - upper_bounds[1]) < 1e-2 or
            abs(yss_fit - lower_bounds[1]) < 1e-2
        )

        delta_throttle = target - base
        K = (yss_fit - y0_fit) / delta_throttle if delta_throttle != 0 else float("nan")

        residuals = y_arr - first_order_model(t_arr, *popt)
        ss_res = np.sum(residuals**2)
        ss_tot = np.sum((y_arr - y_arr.mean())**2)
        r2 = 1 - ss_res/ss_tot if ss_tot > 0 else float("nan")

        warn = "  [!] CANH BAO: fit cham bien, KHONG DANG TIN - can tang captureMs" if near_bound else ""

        line = (f"Step {base:.0f}% -> {target:.0f}%: "
                f"G(s) = {K:.4f} / ({tau_fit:.4f}s + 1)   "
                f"[y0={y0_fit:.2f}g, yss={yss_fit:.2f}g, tau={tau_fit*1000:.1f}ms, R2={r2:.3f}]{warn}")
    except RuntimeError:
        line = f"Step {base:.0f}% -> {target:.0f}%: fit khong hoi tu"

    print(line)
    tf_lines.append(line)

with open(TF_TXT, "w", encoding="utf-8") as f:
    f.write("\n".join(tf_lines) + "\n")

print(f"Da luu ham truyen dong co vao {TF_TXT}")

if fig_sweep is not None:
    fig_sweep.savefig("sweep_plot.png", dpi=200, bbox_inches="tight")
    print("Da luu bieu do sweep vao sweep_plot.png")

if fig_step is not None:
    fig_step.savefig("step_plot.png", dpi=200, bbox_inches="tight")
    print("Da luu bieu do step vao step_plot.png")

plt.ioff()
plt.show()