#!/usr/bin/env python3
import os
import time
from collections import deque

import numpy as np
import matplotlib.pyplot as plt

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from geometry_msgs.msg import Twist, TwistStamped
from geometry_msgs.msg import AccelWithCovarianceStamped
from nav_msgs.msg import Odometry
from autoware_auto_vehicle_msgs.msg import VelocityReport
from pacmod3_msgs.msg import GlobalRpt, VehicleSpeedRpt
from tier4_debug_msgs.msg import Float64Stamped


def get_enable_value(global_rpt_msg):
    for name in ("enabled", "enable", "enabled_status", "enabled_feedback"):
        if hasattr(global_rpt_msg, name):
            val = getattr(global_rpt_msg, name)
            if isinstance(val, bool):
                return val
            if hasattr(val, "data") and isinstance(val.data, bool):
                return val.data
    return None


def stamp_to_sec(stamp):
    return float(stamp.sec) + (float(stamp.nanosec) * 1e-9)


def interp_to_grid(t_src, y_src, t_grid):
    t_src = np.asarray(t_src, dtype=float)
    y_src = np.asarray(y_src, dtype=float)
    if len(t_src) < 2:
        return np.full_like(t_grid, np.nan, dtype=float)
    order = np.argsort(t_src)
    t_src = t_src[order]
    y_src = y_src[order]
    y = np.interp(t_grid, t_src, y_src)
    y[(t_grid < t_src[0]) | (t_grid > t_src[-1])] = np.nan
    return y


def median_dt(tarr):
    tarr = np.asarray(tarr, dtype=float)
    if len(tarr) < 3:
        return None
    d = np.diff(np.sort(tarr))
    d = d[(d > 1e-6) & (d < 1.0)]
    if len(d) == 0:
        return None
    return float(np.median(d))


def y_at_time(t, y, t_query):
    if t_query is None or len(t) < 2:
        return np.nan
    if t_query < t[0] or t_query > t[-1]:
        return np.nan
    mask = ~np.isnan(y)
    if np.sum(mask) < 2:
        return np.nan
    return float(np.interp(t_query, t[mask], y[mask]))


def plot_ts(save_path, x, ys, labels, title, ylab, event_time=None, event_label=None):
    plt.figure()
    for y, lab in zip(ys, labels):
        plt.plot(x, y, label=lab)

    if event_time is not None:
        plt.axvline(event_time, linestyle="--", label=event_label)
        for y in ys:
            yq = y_at_time(x, y, event_time)
            if not np.isnan(yq):
                plt.scatter([event_time], [yq], zorder=5)

    plt.xlabel("Time (s)")
    plt.ylabel(ylab)
    plt.title(title)
    plt.grid(True, alpha=0.3)
    if len(labels) > 1 or event_time is not None:
        plt.legend()

    plt.savefig(save_path, dpi=300, bbox_inches="tight")
    plt.close()


class SaveOnlyAccPlotter(Node):
    def __init__(self):
        super().__init__("save_only_acc_plotter")

        # params
        self.declare_parameter("save_dir", "acc_plots")
        self.declare_parameter("max_buffer_sec", 300.0)  # how much to keep in memory (10 min default)
        self.declare_parameter("plot_refresh_hz", 10.0)

        self.save_dir = str(self.get_parameter("save_dir").value)
        self.max_buffer_sec = float(self.get_parameter("max_buffer_sec").value)
        self.plot_refresh_hz = float(self.get_parameter("plot_refresh_hz").value)

        os.makedirs(self.save_dir, exist_ok=True)

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=200,
        )

        # store as deques of (t, y)
        self.gap = deque()
        self.leader_v = deque()
        self.target_v = deque()
        self.target_a = deque()
        self.ego_v = deque()
        self.localization_v = deque()
        self.pacmod_v = deque()
        self.ego_a = deque()

        # pacmod enable edge detection
        self.prev_enable = None
        self.manual_intervention_time = None

        # live plot state
        self.fig = None
        self.axs = None
        self.live_lines = {}
        self.event_lines = []
        self.plot_window_closed = False
        self.live_plot_enabled = False
        self.plotting_stopped = False
        self.headerless_topic_warnings = set()

        # subscribers
        self.create_subscription(Float64Stamped, "/acc/perception/gap", self.cb_gap, qos)
        self.create_subscription(Float64Stamped, "/acc/perception/velocity", self.cb_leader_v, qos)
        self.create_subscription(TwistStamped, "/acc/target_vel", self.cb_target, qos)
        self.create_subscription(Odometry, "/localization/kinematic_state", self.cb_localization_v, qos)
        self.create_subscription(VehicleSpeedRpt, "/pacmod/vehicle_speed_rpt", self.cb_pacmod_speed, qos)
        self.create_subscription(VelocityReport, "/vehicle/status/velocity_status", self.cb_ego_v, qos)
        self.create_subscription(AccelWithCovarianceStamped, "/localization/acceleration", self.cb_ego_a, qos)
        self.create_subscription(GlobalRpt, "/pacmod/global_rpt", self.cb_pacmod, qos)

        self._setup_live_plot()

        if self.live_plot_enabled:
            refresh_period = 1.0 / max(self.plot_refresh_hz, 0.1)
            self.create_timer(refresh_period, self.update_live_plot)

        self.get_logger().info(
            f"Live ACC plotter recording. save_dir={self.save_dir}, plot_refresh_hz={self.plot_refresh_hz:.1f}"
        )

    def now_s(self):
        return float(self.get_clock().now().nanoseconds) * 1e-9

    def msg_time_s(self, msg, topic_name):
        if hasattr(msg, "header") and hasattr(msg.header, "stamp"):
            stamp = msg.header.stamp
            if stamp.sec != 0 or stamp.nanosec != 0:
                return stamp_to_sec(stamp)

        if hasattr(msg, "stamp"):
            stamp = msg.stamp
            if hasattr(stamp, "sec") and hasattr(stamp, "nanosec"):
                if stamp.sec != 0 or stamp.nanosec != 0:
                    return stamp_to_sec(stamp)

        if topic_name not in self.headerless_topic_warnings:
            self.headerless_topic_warnings.add(topic_name)
            self.get_logger().warn(
                f"{topic_name} has no usable header stamp; falling back to local ROS clock receive time."
            )
        return self.now_s()

    def _trim_old(self, dq, t_now=None):
        """Keep only last max_buffer_sec seconds to avoid unbounded RAM."""
        if self.max_buffer_sec <= 0:
            return
        if t_now is None:
            t_now = self.now_s()
        while len(dq) > 0 and (t_now - dq[0][0]) > self.max_buffer_sec:
            dq.popleft()

    # callbacks
    def cb_gap(self, msg: Float64Stamped):
        t = self.msg_time_s(msg, "/acc/perception/gap")
        self.gap.append((t, float(msg.data)))
        self._trim_old(self.gap, t)

    def cb_leader_v(self, msg: Float64Stamped):
        t = self.msg_time_s(msg, "/acc/perception/velocity")
        self.leader_v.append((t, float(msg.data)))
        self._trim_old(self.leader_v, t)

    def cb_target(self, msg: TwistStamped):
        t = self.msg_time_s(msg, "/acc/target_vel")
        self.target_v.append((t, float(msg.twist.linear.x)))
        self.target_a.append((t, float(msg.twist.linear.z)))
        self._trim_old(self.target_v, t)
        self._trim_old(self.target_a, t)

    def cb_localization_v(self, msg: Odometry):
        t = self.msg_time_s(msg, "/localization/kinematic_state")
        self.localization_v.append((t, float(msg.twist.twist.linear.x)))
        self._trim_old(self.localization_v, t)

    def cb_pacmod_speed(self, msg: VehicleSpeedRpt):
        if hasattr(msg, "vehicle_speed_valid") and not bool(msg.vehicle_speed_valid):
            return
        t = self.msg_time_s(msg, "/pacmod/vehicle_speed_rpt")
        self.pacmod_v.append((t, float(msg.vehicle_speed)))
        self._trim_old(self.pacmod_v, t)

    def cb_ego_v(self, msg: VelocityReport):
        t = self.msg_time_s(msg, "/vehicle/status/velocity_status")
        self.ego_v.append((t, float(msg.longitudinal_velocity)))
        self._trim_old(self.ego_v, t)

    def cb_ego_a(self, msg: AccelWithCovarianceStamped):
        t = self.msg_time_s(msg, "/localization/acceleration")
        self.ego_a.append((t, float(msg.accel.accel.linear.x)))
        self._trim_old(self.ego_a, t)

    def cb_pacmod(self, msg: GlobalRpt):
        en = get_enable_value(msg)
        if en is None:
            return

        if self.manual_intervention_time is None and self.prev_enable is not None:
            if (self.prev_enable is True) and (en is False):
                self.manual_intervention_time = self.msg_time_s(msg, "/pacmod/global_rpt")
                self.get_logger().warn("Manual intervention detected; freezing plots at the message timestamp.")
                self.update_live_plot()
                self.plotting_stopped = True
                self.get_logger().info("Live plotting stopped at manual intervention.")

        self.prev_enable = bool(en)

    def _to_arrays(self, dq):
        if len(dq) == 0:
            return np.array([]), np.array([])
        arr = np.array(dq, dtype=float)
        return arr[:, 0], arr[:, 1]

    def _build_plot_state(self):
        gap_t, gap = self._to_arrays(self.gap)
        leader_t, leader = self._to_arrays(self.leader_v)
        tv_t, target_vel = self._to_arrays(self.target_v)
        ta_t, target_acc = self._to_arrays(self.target_a)
        loc_t, localization_vel = self._to_arrays(self.localization_v)
        pac_t, pacmod_vel = self._to_arrays(self.pacmod_v)
        ev_t, ego_vel = self._to_arrays(self.ego_v)
        ea_t, ego_acc = self._to_arrays(self.ego_a)

        time_arrays = [a for a in [gap_t, leader_t, tv_t, ta_t, loc_t, pac_t, ev_t, ea_t] if a.size > 0]
        if not time_arrays:
            return None

        all_times = np.concatenate(time_arrays)
        t_min_abs = float(np.min(all_times))
        t_max_abs = float(np.max(all_times))

        dt = (
            median_dt(ev_t)
            or median_dt(loc_t)
            or median_dt(pac_t)
            or median_dt(tv_t)
            or median_dt(ea_t)
            or median_dt(ta_t)
            or 0.05
        )
        dt = max(float(dt), 1e-3)

        if t_max_abs <= t_min_abs:
            t_grid = np.array([t_min_abs], dtype=float)
        else:
            t_grid = np.arange(t_min_abs, t_max_abs + dt, dt, dtype=float)
            if t_grid.size == 0:
                t_grid = np.array([t_min_abs, t_max_abs], dtype=float)

        gap_i = interp_to_grid(gap_t, gap, t_grid)
        leader_i = interp_to_grid(leader_t, leader, t_grid)
        target_vel_i = interp_to_grid(tv_t, target_vel, t_grid)
        target_acc_i = interp_to_grid(ta_t, target_acc, t_grid)
        localization_vel_i = interp_to_grid(loc_t, localization_vel, t_grid)
        pacmod_vel_i = interp_to_grid(pac_t, pacmod_vel, t_grid)
        ego_vel_i = interp_to_grid(ev_t, ego_vel, t_grid)
        ego_acc_i = interp_to_grid(ea_t, ego_acc, t_grid)

        if self.manual_intervention_time is not None:
            cutoff = float(self.manual_intervention_time)

            def clip_series(t_arr, y_arr):
                mask = t_arr <= cutoff
                return t_arr[mask], y_arr[mask]

            gap_t, gap = clip_series(gap_t, gap)
            leader_t, leader = clip_series(leader_t, leader)
            tv_t, target_vel = clip_series(tv_t, target_vel)
            ta_t, target_acc = clip_series(ta_t, target_acc)
            loc_t, localization_vel = clip_series(loc_t, localization_vel)
            pac_t, pacmod_vel = clip_series(pac_t, pacmod_vel)
            ev_t, ego_vel = clip_series(ev_t, ego_vel)
            ea_t, ego_acc = clip_series(ea_t, ego_acc)

            grid_mask = t_grid <= cutoff
            t_grid = t_grid[grid_mask]
            gap_i = gap_i[grid_mask]
            leader_i = leader_i[grid_mask]
            target_vel_i = target_vel_i[grid_mask]
            target_acc_i = target_acc_i[grid_mask]
            localization_vel_i = localization_vel_i[grid_mask]
            pacmod_vel_i = pacmod_vel_i[grid_mask]
            ego_vel_i = ego_vel_i[grid_mask]
            ego_acc_i = ego_acc_i[grid_mask]

        plotted_times = [
            a for a in [gap_t, leader_t, tv_t, ta_t, loc_t, pac_t, ev_t, ea_t, t_grid] if a.size > 0
        ]
        if not plotted_times:
            return None

        t_ref = float(min(np.min(a) for a in plotted_times))

        def rel_time(t_arr):
            if t_arr.size == 0:
                return t_arr
            return t_arr - t_ref

        event_time = None
        if self.manual_intervention_time is not None:
            event_time = float(self.manual_intervention_time) - t_ref

        return {
            "gap_t": rel_time(gap_t),
            "gap": gap,
            "leader_t": rel_time(leader_t),
            "leader": leader,
            "tv_t": rel_time(tv_t),
            "target_vel": target_vel,
            "ta_t": rel_time(ta_t),
            "target_acc": target_acc,
            "loc_t": rel_time(loc_t),
            "localization_vel": localization_vel,
            "pac_t": rel_time(pac_t),
            "pacmod_vel": pacmod_vel,
            "ev_t": rel_time(ev_t),
            "ego_vel": ego_vel,
            "ea_t": rel_time(ea_t),
            "ego_acc": ego_acc,
            "t_grid": rel_time(t_grid),
            "gap_i": gap_i,
            "leader_i": leader_i,
            "target_vel_i": target_vel_i,
            "target_acc_i": target_acc_i,
            "localization_vel_i": localization_vel_i,
            "pacmod_vel_i": pacmod_vel_i,
            "ego_vel_i": ego_vel_i,
            "ego_acc_i": ego_acc_i,
            "vel_err": target_vel_i - ego_vel_i,
            "acc_err": target_acc_i - ego_acc_i,
            "event_time": event_time,
            "t_min": 0.0,
            "t_max": float(np.max(rel_time(t_grid))) if t_grid.size > 0 else 0.0,
            "all_times_size": int(all_times.size),
        }

    def _setup_live_plot(self):
        try:
            plt.ion()
            self.fig, self.axs = plt.subplots(7, 1, figsize=(12, 21), sharex=True, num="ACC Live Plotter")

            manager = getattr(self.fig.canvas, "manager", None)
            if manager is not None and hasattr(manager, "set_window_title"):
                manager.set_window_title("ACC Live Plotter")

            self.fig.canvas.mpl_connect("close_event", self._on_plot_closed)

            self.live_lines["ego_vel"] = self.axs[0].plot([], [], label="ego Velocity")[0]
            self.live_lines["target_vel"] = self.axs[0].plot([], [], label="Target Velocity")[0]
            self.live_lines["ego_acc"] = self.axs[1].plot([], [], label="ego Acceleration")[0]
            self.live_lines["target_acc"] = self.axs[1].plot([], [], label="Target Acceleration")[0]
            self.live_lines["vel_err"] = self.axs[2].plot([], [], label="Velocity Tracking Error")[0]
            self.live_lines["acc_err"] = self.axs[3].plot([], [], label="Acceleration Error")[0]
            self.live_lines["gap"] = self.axs[4].plot([], [], label="Gap")[0]
            self.live_lines["leader"] = self.axs[5].plot([], [], label="Leader Velocity")[0]
            self.live_lines["localization_vel"] = self.axs[6].plot([], [], label="/localization/kinematic_state")[0]
            self.live_lines["pacmod_vel"] = self.axs[6].plot([], [], label="/pacmod/vehicle_speed_rpt")[0]
            self.live_lines["vehicle_status_vel"] = self.axs[6].plot([], [], label="/vehicle/status/velocity_status")[0]

            axis_specs = [
                ("ego vs Target Velocity", "Velocity (m/s)"),
                ("ego vs Target Acceleration", "Acceleration (m/s²)"),
                ("Velocity Tracking Error", "Error (m/s)"),
                ("Acceleration Error", "Error (m/s²)"),
                ("Actual Gap", "Gap (m)"),
                ("Leader Vehicle Velocity", "Velocity (m/s)"),
                ("Velocity Source Comparison", "Velocity (m/s)"),
            ]

            for ax, (title, ylabel) in zip(self.axs, axis_specs):
                ax.set_title(title)
                ax.set_ylabel(ylabel)
                ax.grid(True, alpha=0.3)
                ax.legend(loc="upper right")
                event_line = ax.axvline(0.0, linestyle="--", color="tab:red")
                event_line.set_visible(False)
                self.event_lines.append(event_line)

            self.axs[6].set_xlabel("Time (s)")
            self.fig.tight_layout()
            plt.show(block=False)
            plt.pause(0.001)
            self.live_plot_enabled = True
            self.get_logger().info("Opened live plot window.")
        except Exception as exc:
            self.live_plot_enabled = False
            self.get_logger().error(f"Failed to initialize live plot window: {exc}")

    def _on_plot_closed(self, _event):
        self.plot_window_closed = True
        self.get_logger().info("Live plot window closed; continuing to record data.")

    def _set_axis_limits_from_lines(self, ax, line_keys, x_min, x_max):
        y_values = []
        for key in line_keys:
            x_data = np.asarray(self.live_lines[key].get_xdata(), dtype=float)
            y_data = np.asarray(self.live_lines[key].get_ydata(), dtype=float)
            if x_data.size == 0 or y_data.size == 0:
                continue
            mask = np.isfinite(x_data) & np.isfinite(y_data) & (x_data >= x_min) & (x_data <= x_max)
            if np.any(mask):
                y_values.append(y_data[mask])

        if not y_values:
            return

        y_all = np.concatenate(y_values)
        y_min = float(np.min(y_all))
        y_max = float(np.max(y_all))
        if np.isclose(y_min, y_max):
            pad = max(abs(y_min) * 0.1, 0.5)
        else:
            pad = 0.1 * (y_max - y_min)
        ax.set_ylim(y_min - pad, y_max + pad)

    def update_live_plot(self):
        if not self.live_plot_enabled or self.plot_window_closed or self.fig is None:
            return

        if not plt.fignum_exists(self.fig.number):
            self.plot_window_closed = True
            return

        if self.plotting_stopped:
            return

        state = self._build_plot_state()
        if state is None:
            plt.pause(0.001)
            return

        self.live_lines["ego_vel"].set_data(state["ev_t"], state["ego_vel"])
        self.live_lines["target_vel"].set_data(state["tv_t"], state["target_vel"])
        self.live_lines["ego_acc"].set_data(state["ea_t"], state["ego_acc"])
        self.live_lines["target_acc"].set_data(state["ta_t"], state["target_acc"])
        self.live_lines["vel_err"].set_data(state["t_grid"], state["vel_err"])
        self.live_lines["acc_err"].set_data(state["t_grid"], state["acc_err"])
        self.live_lines["gap"].set_data(state["gap_t"], state["gap"])
        self.live_lines["leader"].set_data(state["leader_t"], state["leader"])
        self.live_lines["localization_vel"].set_data(state["loc_t"], state["localization_vel"])
        self.live_lines["pacmod_vel"].set_data(state["pac_t"], state["pacmod_vel"])
        self.live_lines["vehicle_status_vel"].set_data(state["ev_t"], state["ego_vel"])

        if self.max_buffer_sec > 0:
            x_min = max(0.0, state["t_max"] - self.max_buffer_sec)
        else:
            x_min = state["t_min"]
        x_max = max(state["t_max"], x_min + 1.0)

        for ax in self.axs:
            ax.set_xlim(x_min, x_max)

        self._set_axis_limits_from_lines(self.axs[0], ["ego_vel", "target_vel"], x_min, x_max)
        self._set_axis_limits_from_lines(self.axs[1], ["ego_acc", "target_acc"], x_min, x_max)
        self._set_axis_limits_from_lines(self.axs[2], ["vel_err"], x_min, x_max)
        self._set_axis_limits_from_lines(self.axs[3], ["acc_err"], x_min, x_max)
        self._set_axis_limits_from_lines(self.axs[4], ["gap"], x_min, x_max)
        self._set_axis_limits_from_lines(self.axs[5], ["leader"], x_min, x_max)
        self._set_axis_limits_from_lines(
            self.axs[6], ["localization_vel", "pacmod_vel", "vehicle_status_vel"], x_min, x_max
        )

        for event_line in self.event_lines:
            if state["event_time"] is None:
                event_line.set_visible(False)
                continue
            event_line.set_xdata([state["event_time"], state["event_time"]])
            event_line.set_visible(True)

        self.fig.canvas.draw_idle()
        self.fig.canvas.flush_events()
        plt.pause(0.001)

    def save_plots(self):
        state = self._build_plot_state()
        if state is None or state["all_times_size"] < 2:
            self.get_logger().error("Not enough data to save plots.")
            return

        event_time = state["event_time"]
        event_label = "Manual Intervention" if event_time is not None else None
        tag = time.strftime("%Y%m%d_%H%M%S")

        # individual plots
        plot_ts(
            os.path.join(self.save_dir, f"velocity_tracking_{tag}.png"),
            state["t_grid"], [state["ego_vel_i"], state["target_vel_i"]],
            ["ego Velocity", "Target Velocity"],
            "ego vs Target Velocity", "Velocity (m/s)",
            event_time, event_label
        )

        plot_ts(
            os.path.join(self.save_dir, f"acceleration_tracking_{tag}.png"),
            state["t_grid"], [state["ego_acc_i"], state["target_acc_i"]],
            ["ego Acceleration", "Target Acceleration"],
            "ego vs Target Acceleration", "Acceleration (m/s²)",
            event_time, event_label
        )

        plot_ts(
            os.path.join(self.save_dir, f"velocity_error_{tag}.png"),
            state["t_grid"], [state["vel_err"]],
            ["Velocity Tracking Error"],
            "Velocity Tracking Error", "Error (m/s)",
            event_time, event_label
        )

        plot_ts(
            os.path.join(self.save_dir, f"acceleration_error_{tag}.png"),
            state["t_grid"], [state["acc_err"]],
            ["Acceleration Error"],
            "Acceleration Error", "Error (m/s²)",
            event_time, event_label
        )

        plot_ts(
            os.path.join(self.save_dir, f"gap_{tag}.png"),
            state["t_grid"], [state["gap_i"]],
            ["Gap"],
            "Actual Gap", "Gap (m)",
            event_time, event_label
        )

        plot_ts(
            os.path.join(self.save_dir, f"leader_velocity_{tag}.png"),
            state["t_grid"], [state["leader_i"]],
            ["Leader Velocity"],
            "Leader Vehicle Velocity", "Velocity (m/s)",
            event_time, event_label
        )

        plot_ts(
            os.path.join(self.save_dir, f"velocity_source_comparison_{tag}.png"),
            state["t_grid"],
            [state["localization_vel_i"], state["pacmod_vel_i"], state["ego_vel_i"]],
            [
                "/localization/kinematic_state",
                "/pacmod/vehicle_speed_rpt",
                "/vehicle/status/velocity_status",
            ],
            "Velocity Source Comparison",
            "Velocity (m/s)",
            event_time,
            event_label,
        )

        # combined stacked plot
        fig, axs = plt.subplots(7, 1, figsize=(12, 21), sharex=True)

        def add_event(ax, x, ys):
            if event_time is None:
                return
            ax.axvline(event_time, linestyle="--", label=event_label)
            for y in ys:
                yq = y_at_time(x, y, event_time)
                if not np.isnan(yq):
                    ax.scatter([event_time], [yq], zorder=5)

        axs[0].plot(state["t_grid"], state["ego_vel_i"], label="ego Velocity")
        axs[0].plot(state["t_grid"], state["target_vel_i"], label="Target Velocity")
        add_event(axs[0], state["t_grid"], [state["ego_vel_i"], state["target_vel_i"]])
        axs[0].set_ylabel("Velocity (m/s)")
        axs[0].set_title("ego vs Target Velocity")
        axs[0].grid(True, alpha=0.3)
        axs[0].legend()

        axs[1].plot(state["t_grid"], state["ego_acc_i"], label="ego Acceleration")
        axs[1].plot(state["t_grid"], state["target_acc_i"], label="Target Acceleration")
        add_event(axs[1], state["t_grid"], [state["ego_acc_i"], state["target_acc_i"]])
        axs[1].set_ylabel("Acceleration (m/s²)")
        axs[1].set_title("ego vs Target Acceleration")
        axs[1].grid(True, alpha=0.3)
        axs[1].legend()

        axs[2].plot(state["t_grid"], state["vel_err"], label="Velocity Tracking Error")
        add_event(axs[2], state["t_grid"], [state["vel_err"]])
        axs[2].set_ylabel("Error (m/s)")
        axs[2].set_title("Velocity Tracking Error")
        axs[2].grid(True, alpha=0.3)
        axs[2].legend()

        axs[3].plot(state["t_grid"], state["acc_err"], label="Acceleration Error")
        add_event(axs[3], state["t_grid"], [state["acc_err"]])
        axs[3].set_ylabel("Error (m/s²)")
        axs[3].set_title("Acceleration Error")
        axs[3].grid(True, alpha=0.3)
        axs[3].legend()

        axs[4].plot(state["t_grid"], state["gap_i"], label="Gap")
        add_event(axs[4], state["t_grid"], [state["gap_i"]])
        axs[4].set_ylabel("Gap (m)")
        axs[4].set_title("Actual Gap")
        axs[4].grid(True, alpha=0.3)
        axs[4].legend()

        axs[5].plot(state["t_grid"], state["leader_i"], label="Leader Velocity")
        add_event(axs[5], state["t_grid"], [state["leader_i"]])
        axs[5].set_ylabel("Velocity (m/s)")
        axs[5].set_title("Leader Vehicle Velocity")
        axs[5].grid(True, alpha=0.3)
        axs[5].legend()

        axs[6].plot(state["t_grid"], state["localization_vel_i"], label="/localization/kinematic_state")
        axs[6].plot(state["t_grid"], state["pacmod_vel_i"], label="/pacmod/vehicle_speed_rpt")
        axs[6].plot(state["t_grid"], state["ego_vel_i"], label="/vehicle/status/velocity_status")
        add_event(
            axs[6],
            state["t_grid"],
            [state["localization_vel_i"], state["pacmod_vel_i"], state["ego_vel_i"]],
        )
        axs[6].set_xlabel("Time (s)")
        axs[6].set_ylabel("Velocity (m/s)")
        axs[6].set_title("Velocity Source Comparison")
        axs[6].grid(True, alpha=0.3)
        axs[6].legend()

        plt.tight_layout()
        combined_path = os.path.join(self.save_dir, f"all_plots_stacked_{tag}.png")
        fig.savefig(combined_path, dpi=300, bbox_inches="tight")
        plt.close(fig)

        self.get_logger().info(f"Saved plots to: {self.save_dir}")


def main():
    rclpy.init()
    node = SaveOnlyAccPlotter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.save_plots()
        except Exception as e:
            node.get_logger().error(f"Failed to save plots: {e}")
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
