"""Create handoff plots and summary from SIH ULogs, using native sample timing."""

import argparse
import csv
import hashlib
import json
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from pyulog import ULog

COLORS = {"Stock": "#155b91", "PR": "#bd510d"}
EXPECTED_TOPICS = Path(__file__).with_name("logger_topics.txt")


def pitch(data, field, transition_frame):
    q = np.column_stack([data[f"{field}[{i}]"] for i in range(4)]).astype(float)
    q /= np.linalg.norm(q, axis=1)[:, None]
    # Fixed-wing axes relative to the PX4 hover frame: right multiply by Ry(+90°).
    w, x, y, z = q[transition_frame].T
    q[transition_frame] = np.column_stack((w - y, x - z, w + y, z + x)) / np.sqrt(2)
    w, x, y, z = q.T
    return np.degrees(np.arcsin(np.clip(2 * (w * y - z * x), -1, 1)))


def first_at(t, threshold):
    return np.flatnonzero(t >= threshold)[0]


def metric_window(t, left, right):
    return (t >= left) & (t < right)


class Run:
    def __init__(self, label, log_path, expected_commit):
        self.label = label
        self.path = log_path
        self.log = ULog(str(log_path))
        self.commit = self.log.msg_info_dict["ver_sw"]
        if self.commit != expected_commit:
            raise ValueError(f"Unexpected firmware {self.commit} in {log_path}")
        self.topics = {(d.name, d.multi_id): d.data for d in self.log.data_list}
        self.vtol = self.topics["vtol_vehicle_status", 0]
        states = self.vtol["vehicle_vtol_state"]
        self.entry = int(self.vtol["timestamp"][np.flatnonzero(states == 4)[0]])
        self.front = int(self.vtol["timestamp"][np.flatnonzero(states == 1)[0]])
        for line in EXPECTED_TOPICS.read_text().splitlines():
            name, _ = line.split()
            if (name, 0) not in self.topics:
                raise ValueError(f"{label} is missing {name}")

    def time(self, topic, instance=0):
        return (self.topics[topic, instance]["timestamp"].astype(np.int64) - self.entry) / 1e6

    def series(self, topic, field, instance=0):
        return self.time(topic, instance), self.topics[topic, instance][field]

    def routed_pitch(self):
        t = self.time("vehicle_attitude_setpoint")
        return t, pitch(self.topics["vehicle_attitude_setpoint", 0], "q_d", t < 0)

    def fw_pitch(self):
        t = self.time("fw_virtual_attitude_setpoint")
        return t, pitch(self.topics["fw_virtual_attitude_setpoint", 0], "q_d", np.zeros_like(t, dtype=bool))

    def motor_values(self):
        motors = self.topics["actuator_motors", 0]
        if np.any(motors["reversible_flags"]):
            raise ValueError("Motor rail calculation assumes unidirectional outputs")
        return np.column_stack([motors[f"control[{i}]"] for i in range(2)])

    def stats(self):
        route_t, route_pitch = self.routed_pitch()
        bracket_pre = np.flatnonzero(route_t < -.025)[-1]
        bracket_post = first_at(route_t, .05)
        measured_pitch = pitch(self.topics["vehicle_attitude", 0], "q",
                               np.ones(len(self.time("vehicle_attitude")), dtype=bool))
        entry_pitch = measured_pitch[first_at(self.time("vehicle_attitude"), 0)]
        rates_t, rates = self.series("vehicle_angular_velocity", "xyz[1]")
        z_t, z = self.series("vehicle_local_position", "z")
        tecs_t, tecs = self.series("tecs_status", "pitch_sp_rad")
        tecs_first = first_at(tecs_t, 0)
        forward_t, forward_q = self.fw_pitch()
        is_nominal = "nominal" in self.path.stem
        shared_t, route_indices, fw_indices = np.intersect1d(route_t, forward_t, return_indices=True)
        match = first_at(shared_t, -.008 if is_nominal else 0)
        first = route_indices[match]
        forward_first = fw_indices[match]
        routed_q = self.topics["vehicle_attitude_setpoint", 0]
        virtual_q = self.topics["fw_virtual_attitude_setpoint", 0]
        a = np.array([routed_q[f"q_d[{i}]"][first] for i in range(4)], dtype=float)
        b = np.array([virtual_q[f"q_d[{i}]"][forward_first] for i in range(4)], dtype=float)
        q_difference = min(np.linalg.norm(a - b), np.linalg.norm(a + b))
        if not is_nominal and (route_t[first] <= .100 or q_difference > 1e-6):
            raise ValueError(f"Delayed handoff did not pass through the fresh FW quaternion in {self.path}")
        first_rates = np.flatnonzero(metric_window(rates_t, 0, 1.5))
        peak = first_rates[np.argmin(rates[first_rates])]
        motor_t = self.time("actuator_motors")
        motor_data = self.motor_values()
        motor_rails = np.any((motor_data <= 1e-6) | (motor_data >= 1-1e-6), axis=1)
        one_to_three = metric_window(motor_t, .6, 3)
        servos_t = self.time("actuator_servos")
        servo_data = np.column_stack([self.topics["actuator_servos", 0][f"control[{i}]"] for i in range(2)])
        servo_rails = np.any(np.abs(servo_data) >= 1-1e-6, axis=1)
        return {
            "commit": self.commit,
            "sha256": hashlib.sha256(self.path.read_bytes()).hexdigest(),
            "dropouts": len(self.log.dropouts),
            "fw_entry_us": self.entry,
            "transition_duration_s": (self.entry - self.front)/1e6,
            "pitch_at_entry_deg": float(entry_pitch),
            "bracket_before_pitch_deg": float(route_pitch[bracket_pre]),
            "bracket_before_ms": round(1000*route_t[bracket_pre], 3),
            "bracket_after_pitch_deg": float(route_pitch[bracket_post]),
            "bracket_after_ms": round(1000*route_t[bracket_post], 3),
            "first_source_matched_routed_fw_pitch_deg": float(pitch(routed_q, "q_d", np.zeros_like(route_t, dtype=bool))[first]),
            "first_source_matched_fw_setpoint_ms": round(1000*route_t[first], 3),
            "nominal_pitch_change_bracket_deg": abs(float(route_pitch[bracket_post] - route_pitch[bracket_pre]))
            if is_nominal else None,
            "first_tecs_ms": round(1000*tecs_t[tecs_first], 3),
            "first_tecs_pitch_deg": float(np.degrees(tecs[tecs_first])),
            "first_fw_virtual_ms": round(1000*forward_t[forward_first], 3),
            "first_fw_virtual_pitch_deg": float(forward_q[forward_first]),
            "first_routed_virtual_quaternion_difference_norm": float(q_difference),
            "peak_nose_down_first_1p5s_deg_s": -float(np.degrees(rates[peak])),
            "peak_nose_down_ms": round(1000*rates_t[peak], 3),
            "height_loss_after_entry_3s_m": float(np.max(z[metric_window(z_t, 0, 3)]) - np.interp(0, z_t, z)),
            "motor_rail_samples_0p6_to_3s": int(np.count_nonzero(motor_rails & one_to_three)),
            "motor_samples_0p6_to_3s": int(np.count_nonzero(one_to_three)),
            "servo_rail_samples_0p6_to_3s": int(np.count_nonzero(servo_rails & metric_window(servos_t, .6, 3))),
        }

    def cadence(self):
        rows = []
        for (name, instance), data in sorted(self.topics.items()):
            t = self.time(name, instance)
            selected = t[metric_window(t, 0, 3)]
            intervals = np.diff(selected) * 1000
            rows.append({"topic": name, "instance": instance, "samples_first_3s": len(selected),
                         "first_after_entry_ms": round(1000*selected[0], 3) if len(selected) else None,
                         "median_interval_ms": round(float(np.median(intervals)), 3) if len(intervals) else None,
                         "max_interval_ms": round(float(np.max(intervals)), 3) if len(intervals) else None})
        return rows

    def write_series(self, path):
        columns = ["topic", "instance", "since_fw_s", "time_us", "signal", "value"]
        with path.open("w", newline="") as stream:
            writer = csv.writer(stream)
            writer.writerow(columns)
            for (name, instance), data in sorted(self.topics.items()):
                if name == "event":
                    continue
                t = self.time(name, instance)
                for i in np.flatnonzero(metric_window(t, -.5, 3.001)):
                    fields = (f for f in data if f != "timestamp")
                    writer.writerows((name, instance, f"{t[i]:.6f}", int(data["timestamp"][i]), field,
                                      data[field][i].item()) for field in fields)


def trace(ax, t, y, label, color, style="-", width=1.5, window=(-.35, 3.0)):
    keep = metric_window(t, window[0], window[1] + .001)
    t, y = t[keep], y[keep]
    split = np.flatnonzero(np.diff(t) > .04) + 1
    for part, (x, v) in enumerate(zip(np.split(t, split), np.split(y, split))):
        if len(x):
            ax.plot(x, v, ls=style, lw=width, c=color, label=label if part == 0 else "_nolegend_")


def plot_pair(pair, destination, scenario):
    fig, axes = plt.subplots(6, sharex=True, figsize=(10.6, 12.5), layout="constrained")
    window = (-.3, 2.65)
    fig.suptitle(f"PX4 SIH Tailsitter Duo · {scenario} FW handoff", fontsize=18, fontweight="bold")
    for run in pair:
        label = run.label
        color = COLORS[label]
        t, sp = run.routed_pitch()
        # The routed message can carry the source timestamp from before the state switch.
        # Mask the frame boundary. A gap after this band represents missing publications.
        before, after = t < -.025, t >= .025
        trace(axes[0], t[before], sp[before], f"{label} routed pitch", color, width=2.0, window=window)
        trace(axes[0], t[after], sp[after], "_nolegend_", color, width=2.0, window=window)
        tecs_t, psp = run.series("tecs_status", "pitch_sp_rad")
        active = tecs_t >= 0
        trace(axes[0], tecs_t[active], np.degrees(psp[active]), f"{label} TECS", color, "--", window=window)
        if label == "PR":
            fw_t, fw_sp = run.fw_pitch()
            active = fw_t >= 0
            trace(axes[0], fw_t[active], fw_sp[active], "PR FW input", color, ":", window=window)
        att_t = run.time("vehicle_attitude")
        measured = pitch(run.topics["vehicle_attitude", 0], "q", np.ones(len(att_t), dtype=bool))
        trace(axes[1], att_t, measured, label, color, window=window)
        rates_t, body_y = run.series("vehicle_angular_velocity", "xyz[1]")
        trace(axes[2], rates_t, np.degrees(body_y), f"{label} measured", color, window=window)
        rate_t, pitch_rate = run.series("vehicle_rates_setpoint", "pitch")
        trace(axes[2], rate_t, np.degrees(pitch_rate), f"{label} requested", color, "--", window=window)
        z_t, z = run.series("vehicle_local_position", "z")
        trace(axes[3], z_t, -(z-np.interp(0, z_t, z)), label, color, window=window)
        thrust_t, thrust = run.series("vehicle_thrust_setpoint", "xyz[2]")
        trace(axes[4], thrust_t, -thrust, f"{label} routed collective", color, window=window)
        throttle_t, throttle = run.series("tecs_status", "throttle_sp")
        active = throttle_t >= 0
        trace(axes[4], throttle_t[active], throttle[active], f"{label} TECS throttle", color, "--", window=window)
        motor_t = run.time("actuator_motors")
        motors = run.motor_values()
        trace(axes[5], motor_t, np.max(motors, axis=1), f"{label} motor max", color, window=window)
        trace(axes[5], motor_t, np.min(motors, axis=1), f"{label} motor min", color, ":", window=window)
        servo_t = run.time("actuator_servos")
        servo = run.topics["actuator_servos", 0]
        max_servo = np.maximum(np.abs(servo["control[0]"]), np.abs(servo["control[1]"]))
        trace(axes[5], servo_t, max_servo, f"{label} |servo| max", color, "--", window=window)

    for ax, ylabel in zip(axes, ["Pitch request (°)", "Measured pitch (°)", "Body pitch rate (°/s)",
                                "Height from FW entry (m)", "Normalized thrust", "Normalized actuator output"]):
        ax.axvline(0, c="#555555", lw=1, ls=":")
        ax.grid(axis="y", color="#d9e0e5", lw=.7)
        ax.set_ylabel(ylabel)
        ax.set_xlim(*window)
        ax.legend(loc="upper right", fontsize=8, ncol=3, framealpha=.93)
        ax.spines[["top", "right"]].set_visible(False)
    for rail in (0, 1):
        axes[5].axhline(rail, c="#777777", lw=.7, ls=":")
    axes[0].axvspan(-.025, .025, color="#e5e9ed", alpha=.65, zorder=0)
    axes[-1].set_xlabel("Seconds from first FW status publication")
    fig.savefig(destination, dpi=165)
    plt.close(fig)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("bundle", type=Path)
    args = parser.parse_args()
    root = args.bundle
    stock = "5db5c623a15dea085bfb788db587bfadfecec097"
    candidate = "4072b456a39197a8b4c0f6a8651d528df5e9f105"
    all_stats, cadence = [], []
    parameter_diffs = []
    for scenario in ("nominal", "delayed", "delayed-repeat"):
        pair = [Run(label, root / f"{variant}-{scenario}.ulg", ref)
                for variant, label, ref in (("stock", "Stock", stock), ("candidate", "PR", candidate))]
        plot_pair(pair, root / f"{scenario}-handoff.png", scenario.replace("-", " ").title())
        stock_params, pr_params = (run.log.initial_parameters for run in pair)
        for name in sorted(stock_params.keys() | pr_params.keys()):
            a, b = stock_params.get(name), pr_params.get(name)
            if a != b:
                parameter_diffs.append({"scenario": scenario, "param": name, "stock": a, "pr": b})
        for run in pair:
            run_id = run.path.stem
            all_stats.append({"run": run_id, **run.stats()})
            cadence.extend({"run": run_id, **row} for row in run.cadence())
            run.write_series(root / f"{run_id}-native.csv")
    for name, rows in (("summary", all_stats), ("logging-cadence", cadence)):
        with (root / f"{name}.csv").open("w", newline="") as stream:
            writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
            writer.writeheader()
            writer.writerows(rows)
    with (root / "initial-parameter-differences.csv").open("w", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=["scenario", "param", "stock", "pr"])
        writer.writeheader()
        writer.writerows(parameter_diffs)
    (root / "summary.json").write_text(json.dumps(all_stats, indent=2) + "\n")


if __name__ == "__main__":
    main()
