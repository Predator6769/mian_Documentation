#!/usr/bin/env python3
import os
import time
from collections import deque

import numpy as np
import matplotlib.pyplot as plt

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from std_msgs.msg import Float64
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from geometry_msgs.msg import AccelWithCovarianceStamped
from pacmod3_msgs.msg import GlobalRpt


def get_enable_value(global_rpt_msg):
    for name in ("enabled", "enable", "enabled_status", "enabled_feedback"):
        if hasattr(global_rpt_msg, name):
            val = getattr(global_rpt_msg, name)
            if isinstance(val, bool):
                return val
            if hasattr(val, "data") and isinstance(val.data, bool):
                return val.data
    return None


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

        self.save_dir = str(self.get_parameter("save_dir").value)
        self.max_buffer_sec = float(self.get_parameter("max_buffer_sec").value)

        os.makedirs(self.save_dir, exist_ok=True)

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=200,
        )

        self.t0 = time.time()

        # store as deques of (t, y)
        self.gap = deque()
        self.leader_v = deque()
        self.target_v = deque()
        self.target_a = deque()
        self.ego_v = deque()
        self.ego_a = deque()

        # pacmod enable edge detection
        self.prev_enable = None
        self.manual_intervention_time = None

        # subscribers
        self.create_subscription(Float64, "/acc/perception/gap", self.cb_gap, qos)
        self.create_subscription(Float64, "/acc/perception/velocity", self.cb_leader_v, qos)
        self.create_subscription(Twist, "/acc/target_vel", self.cb_target, qos)
        self.create_subscription(Odometry, "/localization/kinematic_state", self.cb_ego_v, qos)
        self.create_subscription(AccelWithCovarianceStamped, "/localization/acceleration", self.cb_ego_a, qos)
        self.create_subscription(GlobalRpt, "/pacmod/global_rpt", self.cb_pacmod, qos)

        self.get_logger().info(f"Save-only plotter recording. save_dir={self.save_dir}")

    def now_s(self):
        return time.time() - self.t0

    def _trim_old(self, dq):
        """Keep only last max_buffer_sec seconds to avoid unbounded RAM."""
        if self.max_buffer_sec <= 0:
            return
        t_now = self.now_s()
        while len(dq) > 0 and (t_now - dq[0][0]) > self.max_buffer_sec:
            dq.popleft()

    # callbacks
    def cb_gap(self, msg: Float64):
        self.gap.append((self.now_s(), float(msg.data)))
        self._trim_old(self.gap)

    def cb_leader_v(self, msg: Float64):
        self.leader_v.append((self.now_s(), float(msg.data)))
        self._trim_old(self.leader_v)

    def cb_target(self, msg: Twist):
        t = self.now_s()
        self.target_v.append((t, float(msg.linear.x)))
        self.target_a.append((t, float(msg.linear.z)))
        self._trim_old(self.target_v)
        self._trim_old(self.target_a)

    def cb_ego_v(self, msg: Odometry):
        self.ego_v.append((self.now_s(), float(msg.twist.twist.linear.x)))
        self._trim_old(self.ego_v)

    def cb_ego_a(self, msg: AccelWithCovarianceStamped):
        self.ego_a.append((self.now_s(), float(msg.accel.accel.linear.x)))
        self._trim_old(self.ego_a)

    def cb_pacmod(self, msg: GlobalRpt):
        en = get_enable_value(msg)
        if en is None:
            return

        if self.manual_intervention_time is None and self.prev_enable is not None:
            if (self.prev_enable is True) and (en is False):
                self.manual_intervention_time = self.now_s()
                self.get_logger().warn(f"Manual intervention detected at t={self.manual_intervention_time:.3f}s")

        self.prev_enable = bool(en)

    def _to_arrays(self, dq):
        if len(dq) == 0:
            return np.array([]), np.array([])
        arr = np.array(dq, dtype=float)
        return arr[:, 0], arr[:, 1]

    def save_plots(self):
        # Pull arrays
        gap_t, gap = self._to_arrays(self.gap)
        leader_t, leader = self._to_arrays(self.leader_v)
        tv_t, target_vel = self._to_arrays(self.target_v)
        ta_t, target_acc = self._to_arrays(self.target_a)
        ev_t, ego_vel = self._to_arrays(self.ego_v)
        ea_t, ego_acc = self._to_arrays(self.ego_a)

        all_times = np.concatenate([a for a in [gap_t, leader_t, tv_t, ta_t, ev_t, ea_t] if a.size > 0])
        if all_times.size < 2:
            self.get_logger().error("Not enough data to save plots.")
            return

        t_min, t_max = float(np.min(all_times)), float(np.max(all_times))
        dt = (
            median_dt(ev_t) or median_dt(tv_t) or median_dt(ea_t) or median_dt(ta_t) or 0.05
        )
        t_grid = np.arange(t_min, t_max, dt)

        gap_i = interp_to_grid(gap_t, gap, t_grid)
        leader_i = interp_to_grid(leader_t, leader, t_grid)
        target_vel_i = interp_to_grid(tv_t, target_vel, t_grid)
        target_acc_i = interp_to_grid(ta_t, target_acc, t_grid)
        ego_vel_i = interp_to_grid(ev_t, ego_vel, t_grid)
        ego_acc_i = interp_to_grid(ea_t, ego_acc, t_grid)

        vel_err = target_vel_i - ego_vel_i
        acc_err = target_acc_i - ego_acc_i

        event_label = "Manual Intervention" if self.manual_intervention_time is not None else None
        tag = time.strftime("%Y%m%d_%H%M%S")

        # individual plots
        plot_ts(
            os.path.join(self.save_dir, f"velocity_tracking_{tag}.png"),
            t_grid, [ego_vel_i, target_vel_i],
            ["ego Velocity", "Target Velocity"],
            "ego vs Target Velocity", "Velocity (m/s)",
            self.manual_intervention_time, event_label
        )

        plot_ts(
            os.path.join(self.save_dir, f"acceleration_tracking_{tag}.png"),
            t_grid, [ego_acc_i, target_acc_i],
            ["ego Acceleration", "Target Acceleration"],
            "ego vs Target Acceleration", "Acceleration (m/s²)",
            self.manual_intervention_time, event_label
        )

        plot_ts(
            os.path.join(self.save_dir, f"velocity_error_{tag}.png"),
            t_grid, [vel_err],
            ["Velocity Tracking Error"],
            "Velocity Tracking Error", "Error (m/s)",
            self.manual_intervention_time, event_label
        )

        plot_ts(
            os.path.join(self.save_dir, f"acceleration_error_{tag}.png"),
            t_grid, [acc_err],
            ["Acceleration Error"],
            "Acceleration Error", "Error (m/s²)",
            self.manual_intervention_time, event_label
        )

        plot_ts(
            os.path.join(self.save_dir, f"gap_{tag}.png"),
            t_grid, [gap_i],
            ["Gap"],
            "Actual Gap", "Gap (m)",
            self.manual_intervention_time, event_label
        )

        plot_ts(
            os.path.join(self.save_dir, f"leader_velocity_{tag}.png"),
            t_grid, [leader_i],
            ["Leader Velocity"],
            "Leader Vehicle Velocity", "Velocity (m/s)",
            self.manual_intervention_time, event_label
        )

        # combined stacked plot
        fig, axs = plt.subplots(6, 1, figsize=(12, 18), sharex=True)

        def add_event(ax, x, ys):
            if self.manual_intervention_time is None:
                return
            ax.axvline(self.manual_intervention_time, linestyle="--", label=event_label)
            for y in ys:
                yq = y_at_time(x, y, self.manual_intervention_time)
                if not np.isnan(yq):
                    ax.scatter([self.manual_intervention_time], [yq], zorder=5)

        axs[0].plot(t_grid, ego_vel_i, label="ego Velocity")
        axs[0].plot(t_grid, target_vel_i, label="Target Velocity")
        add_event(axs[0], t_grid, [ego_vel_i, target_vel_i])
        axs[0].set_ylabel("Velocity (m/s)")
        axs[0].set_title("ego vs Target Velocity")
        axs[0].grid(True, alpha=0.3)
        axs[0].legend()

        axs[1].plot(t_grid, ego_acc_i, label="ego Acceleration")
        axs[1].plot(t_grid, target_acc_i, label="Target Acceleration")
        add_event(axs[1], t_grid, [ego_acc_i, target_acc_i])
        axs[1].set_ylabel("Acceleration (m/s²)")
        axs[1].set_title("ego vs Target Acceleration")
        axs[1].grid(True, alpha=0.3)
        axs[1].legend()

        axs[2].plot(t_grid, vel_err, label="Velocity Tracking Error")
        add_event(axs[2], t_grid, [vel_err])
        axs[2].set_ylabel("Error (m/s)")
        axs[2].set_title("Velocity Tracking Error")
        axs[2].grid(True, alpha=0.3)
        axs[2].legend()

        axs[3].plot(t_grid, acc_err, label="Acceleration Error")
        add_event(axs[3], t_grid, [acc_err])
        axs[3].set_ylabel("Error (m/s²)")
        axs[3].set_title("Acceleration Error")
        axs[3].grid(True, alpha=0.3)
        axs[3].legend()

        axs[4].plot(t_grid, gap_i, label="Gap")
        add_event(axs[4], t_grid, [gap_i])
        axs[4].set_ylabel("Gap (m)")
        axs[4].set_title("Actual Gap")
        axs[4].grid(True, alpha=0.3)
        axs[4].legend()

        axs[5].plot(t_grid, leader_i, label="Leader Velocity")
        add_event(axs[5], t_grid, [leader_i])
        axs[5].set_xlabel("Time (s)")
        axs[5].set_ylabel("Velocity (m/s)")
        axs[5].set_title("Leader Vehicle Velocity")
        axs[5].grid(True, alpha=0.3)
        axs[5].legend()

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