#!/usr/bin/env python3
"""Analyze one recorded session without ROS and save PNGs in picture/<session_id>."""
import argparse
from collections import Counter
import json
from pathlib import Path
import sys
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))


def finite_vector(value, size=3):
    try:
        result = np.asarray(value, dtype=float)
        if result.shape == (size,) and np.isfinite(result).all():
            return result
    except (TypeError, ValueError):
        pass
    return np.full(size, np.nan)


def nearest_indices(target_times, source_times, max_gap=.1):
    """Return -1 for missing/stale records, never silently fill with zero."""
    target = np.asarray(target_times, dtype=float)
    source = np.asarray(source_times, dtype=float)
    result = np.full(len(target), -1, dtype=int)
    if not len(source):
        return result
    if (np.diff(source) < 0).any():
        raise ValueError("source_times must be sorted")
    right = np.searchsorted(source, target)
    left = np.clip(right - 1, 0, len(source) - 1)
    right = np.clip(right, 0, len(source) - 1)
    selected = np.where(np.abs(source[left] - target) <= np.abs(source[right] - target), left, right)
    valid = np.isfinite(target) & (np.abs(source[selected] - target) <= max_gap)
    result[valid] = selected[valid]
    return result


def clean_json(value):
    if isinstance(value, dict):
        return {str(k): clean_json(v) for k, v in value.items()}
    if isinstance(value, (list, tuple, np.ndarray)):
        return [clean_json(v) for v in value]
    if isinstance(value, (float, np.floating)):
        return float(value) if np.isfinite(value) else None
    if isinstance(value, np.integer):
        return int(value)
    if isinstance(value, np.bool_):
        return bool(value)
    return value


def statistics(times, values, tail_seconds=10, threshold=None):
    times, values = np.asarray(times), np.asarray(values, dtype=float)
    valid = np.isfinite(times) & np.isfinite(values)
    tail = (times >= times[-1] - tail_seconds) & valid if len(times) else valid
    result = {"samples": len(values), "valid_samples": int(valid.sum()), "missing_samples": int((~valid).sum()),
              "rmse": float(np.sqrt(np.mean(values[valid]**2))) if valid.any() else None,
              "peak_abs": float(np.abs(values[valid]).max()) if valid.any() else None,
              "tail_rmse": float(np.sqrt(np.mean(values[tail]**2))) if tail.any() else None,
              "tail_max_abs": float(np.abs(values[tail]).max()) if tail.any() else None,
              "threshold": threshold, "sampled_settling_time_s": None}
    if threshold is not None and len(values) and valid.all():
        bad = np.flatnonzero(np.abs(values) > threshold)
        if not len(bad):
            result["sampled_settling_time_s"] = float(times[0])
        elif bad[-1] < len(values) - 1:
            result["sampled_settling_time_s"] = float(times[bad[-1] + 1])
    return result


def load_session(directory):
    directory = Path(directory).resolve()
    metadata = json.loads((directory / "metadata.json").read_text(encoding="utf-8"))
    all_records, malformed = [], 0
    with (directory / "telemetry.jsonl").open(encoding="utf-8") as handle:
        for line in handle:
            try:
                record = json.loads(line)
                if not isinstance(record, dict) or not isinstance(record.get("id"), str):
                    raise ValueError("Missing agent ID")
                stamp = record.get("stamp", record.get("sim_time"))
                if stamp is None or not np.isfinite(float(stamp)):
                    raise ValueError("Invalid timestamp")
                record["stamp"] = float(stamp)
                all_records.append(record)
            except (ValueError, TypeError, json.JSONDecodeError):
                malformed += 1
    return metadata, all_records, malformed


def analyze(directory, picture_root=None, max_gap=.1, tail_seconds=10,
            position_threshold=.05, velocity_threshold=.05):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from ne_tvft.core import MonitoringGame
    directory = Path(directory).resolve()
    metadata, all_records, malformed = load_session(directory)
    config = metadata["config"]
    groups = config["groups"]
    ids = [name for group in groups for name in [group["leader"]] + group["followers"]]
    mapping = {group["leader"]: (i, -1) for i, group in enumerate(groups)}
    mapping.update({name: (i, j) for i, group in enumerate(groups) for j, name in enumerate(group["followers"])})
    sid = metadata.get("session_id", directory.name)
    if sid != directory.name:
        raise ValueError("Session directory name and metadata session_id do not match")
    t0 = float(metadata.get("t0", metadata.get("session", {}).get("t0", 0)) or 0)
    raw = {name: [] for name in ids}
    unknown = Counter()
    phases = Counter()
    for record in all_records:
        phases[record.get("phase", "UNKNOWN")] += 1
        if record.get("session_id", sid) != sid:
            continue
        if record["id"] not in raw:
            unknown[record["id"]] += 1
            continue
        try:
            tau = float(record.get("tau", record["stamp"] - t0))
        except (TypeError, ValueError):
            continue
        if record.get("phase") == "RUNNING" and np.isfinite(tau) and tau >= 0:
            record["tau"] = tau
            raw[record["id"]].append(record)
    for rows in raw.values():
        rows.sort(key=lambda record: record["stamp"])
    if not any(raw.values()):
        raise ValueError("No valid RUNNING telemetry samples at or after t0")
    root = Path(picture_root).expanduser() if picture_root else directory.parent.parent / "picture"
    output = root.resolve() / sid
    output.mkdir(parents=True, exist_ok=True)
    plt.rcParams.update({"font.family": "DejaVu Sans", "font.size": 9, "axes.grid": True,
                         "grid.alpha": .2, "axes.spines.top": False, "axes.spines.right": False,
                         "savefig.dpi": 180})
    colors = ["#2563eb", "#e05a33", "#059669", "#9254c8"]
    figures = []
    metrics = {"session_id": sid, "source": metadata.get("source", "ROS telemetry"),
               "formal_phase": "RUNNING", "t0": t0, "tail_seconds": tail_seconds,
               "matching_max_gap_s": max_gap, "thresholds": {"position_m": position_threshold,
               "velocity_m_s": velocity_threshold}, "agents": {}, "data_quality": {
                   "malformed_lines": malformed, "unknown_agents": dict(unknown), "phase_counts": dict(phases),
                   "missing_agents": [name for name in ids if not raw[name]]}}
    game = MonitoringGame(config)
    equilibrium = np.asarray(game.equilibrium(), dtype=float)
    metrics["equilibrium"] = equilibrium.tolist()
    arrays = {}
    for name, records in raw.items():
        if not records:
            continue
        arrays[name] = {"t": np.asarray([r["tau"] for r in records]),
                        "stamp": np.asarray([r["stamp"] for r in records]),
                        "p": np.stack([finite_vector(r.get("position")) for r in records]),
                        "v": np.stack([finite_vector(r.get("velocity")) for r in records]),
                        "raw": np.stack([finite_vector(r.get("input_raw")) for r in records]),
                        "applied": np.stack([finite_vector(r.get("input_applied")) for r in records]),
                        "accel": np.stack([finite_vector(r.get("acceleration_estimate"))
                                  if r.get("acceleration_valid", False) else np.full(3, np.nan) for r in records]),
                        "imu": np.stack([finite_vector(r.get("imu_specific_force_body")) for r in records]),
                        "limited": np.array([bool(r.get("limited", False)) for r in records]),
                        "controller": [r["controller"] if isinstance(r.get("controller"), dict) else {}
                                       for r in records]}
        data = arrays[name]
        dt = np.diff(data["stamp"])
        actual_offsets = [r.get("start_offset_s") for r in records if r.get("start_offset_s") is not None]
        agent_metrics = {"formal_samples": len(records), "duration_s": float(data["t"][-1] - data["t"][0]),
                         "invalid_positions": int((~np.isfinite(data["p"]).all(axis=1)).sum()),
                         "invalid_velocities": int((~np.isfinite(data["v"]).all(axis=1)).sum()),
                         "valid_acceleration_samples": int(np.isfinite(data["accel"]).all(axis=1).sum()),
                         "limited_fraction": float(data["limited"].mean()),
                         "maximum_sample_gap_s": float(dt.max()) if len(dt) else None,
                         "duplicate_timestamp_count": int((dt == 0).sum()),
                         "start_offset_s": actual_offsets[0] if actual_offsets else None,
                         "position_source_max_age_s": max([max(0.0, r["stamp"] - float(r["pose_stamp"]))
                                         for r in records if r.get("pose_stamp") is not None] or [float("nan")]),
                         "velocity_source_max_age_s": max([max(0.0, r["stamp"] - float(r["velocity_stamp"]))
                                         for r in records if r.get("velocity_stamp") is not None] or [float("nan")])}
        metrics["agents"][name] = agent_metrics

    def save(fig, name):
        fig.savefig(output / (name + ".png"), bbox_inches="tight")
        figures.append(name + ".png")
        plt.close(fig)

    def style_axes(axes, title, ylabel):
        for i, ax in enumerate(np.asarray(axes).flat):
            ax.set(title="Group {}: {}".format(i+1, title), xlabel="Time from t0 (s)", ylabel=ylabel)

    # Complete spatial trajectories of the formal segment, measured in world ENU.
    fig = plt.figure(figsize=(13, 6), constrained_layout=True)
    ax2, ax3 = fig.add_subplot(121), fig.add_subplot(122, projection="3d")
    for name, data in arrays.items():
        i, follower = mapping[name]
        line = "-" if follower < 0 else "--" if follower == 0 else ":"
        ax2.plot(data["p"][:, 0], data["p"][:, 1], line, color=colors[i % len(colors)], label=name, lw=1.3)
        ax3.plot(*data["p"].T, line, color=colors[i % len(colors)], label=name, lw=1.1)
    ax2.scatter(equilibrium[:, 0], equilibrium[:, 1], marker="*", color="black", s=80, label="Offline NE")
    ax2.set(xlabel="World ENU x (m)", ylabel="World ENU y (m)", title="Formal-segment horizontal trajectories")
    ax2.set_aspect("equal", adjustable="box")
    ax2.legend(ncol=2, fontsize=8)
    ax3.set(xlabel="x (m)", ylabel="y (m)", title="Formal-segment 3D trajectories")
    ax3.text2D(.88, .92, "z (m)", transform=ax3.transAxes)
    save(fig, "01_trajectories")

    # Leader diagnostics use independently matched samples, no non-neighbor readings in control.
    fig, axes = plt.subplots(len(groups), 3, figsize=(13, 3*len(groups)), squeeze=False, constrained_layout=True)
    for i, group in enumerate(groups):
        name = group["leader"]
        if name not in arrays:
            continue
        data = arrays[name]
        y = np.stack([finite_vector(c.get("y"), 2) for c in data["controller"]])
        ne_error = np.linalg.norm(data["p"][:, :2] - equilibrium[i], axis=1)
        virtual_error = np.linalg.norm(y - equilibrium[i], axis=1)
        track_error = np.linalg.norm(data["p"][:, :2] - y, axis=1)
        axes[i, 0].semilogy(data["t"], np.maximum(ne_error, 1e-12), label="physical NE")
        axes[i, 0].semilogy(data["t"], np.maximum(virtual_error, 1e-12), label="virtual NE")
        axes[i, 1].semilogy(data["t"], np.maximum(track_error, 1e-12), label="position - virtual reference")
        estimate_errors = np.full((len(data["t"]), len(groups)), np.nan)
        for k, peer_group in enumerate(groups):
            peer = arrays.get(peer_group["leader"])
            if peer is None:
                continue
            matches = nearest_indices(data["stamp"], peer["stamp"], max_gap)
            for j, matched in enumerate(matches):
                if matched < 0:
                    continue
                try:
                    estimate = np.asarray(data["controller"][j]["z"], dtype=float)[k]
                    peer_y = finite_vector(peer["controller"][matched].get("y"), 2)
                    estimate_errors[j, k] = np.linalg.norm(estimate - peer_y)
                except (KeyError, ValueError, IndexError, TypeError):
                    pass
            axes[i, 2].semilogy(data["t"], np.maximum(estimate_errors[:, k], 1e-12), label="z for " + peer_group["leader"])
        for j, label in enumerate(["NE errors", "Reference tracking", "Local estimation errors"]):
            axes[i, j].set(title=name + ": " + label, xlabel="Time from t0 (s)", ylabel="Error norm (m)")
            axes[i, j].legend(fontsize=7)
        metrics["agents"][name].update({"ne_error_m": statistics(data["t"], ne_error, tail_seconds, position_threshold),
                                      "reference_error_m": statistics(data["t"], track_error, tail_seconds, position_threshold),
                                      "virtual_ne_error_m": statistics(data["t"], virtual_error, tail_seconds, position_threshold),
                                      "estimator_errors_m": [statistics(data["t"], estimate_errors[:, k], tail_seconds, position_threshold)
                                                             for k in range(len(groups))]})
    save(fig, "02_leader_diagnostics")

    # Relative UAV motion and formation errors require measured leader position and velocity.
    relative = {}
    for name, data in arrays.items():
        i, follower = mapping[name]
        if follower < 0:
            continue
        leader = arrays.get(groups[i]["leader"])
        matches = nearest_indices(data["stamp"], leader["stamp"], max_gap) if leader is not None else np.full(len(data["t"]), -1)
        relp, relv = np.full((len(matches), 2), np.nan), np.full((len(matches), 2), np.nan)
        valid = matches >= 0
        if valid.any():
            relp[valid] = data["p"][valid, :2] - leader["p"][matches[valid], :2]
            relv[valid] = data["v"][valid, :2] - leader["v"][matches[valid], :2]
        radius = np.linalg.norm(relp, axis=1)
        phase = np.arctan2(relp[:, 1], relp[:, 0])
        omega = np.divide(relp[:, 0]*relv[:, 1] - relp[:, 1]*relv[:, 0], radius**2,
                          out=np.full(len(radius), np.nan), where=radius > 1e-6)
        desired_radius = float(config["formation"]["radii"][i])
        desired_omega = float(config["formation"]["angular_rates"][i])
        desired_phase = desired_omega*data["t"] + config["formation"]["phases"][i] + follower*np.pi
        phase_error = np.arctan2(np.sin(phase-desired_phase), np.cos(phase-desired_phase))
        h = desired_radius*np.column_stack([np.cos(desired_phase), np.sin(desired_phase)])
        dh = desired_omega*np.column_stack([-h[:, 1], h[:, 0]])
        error_position, error_velocity = np.linalg.norm(relp-h, axis=1), np.linalg.norm(relv-dh, axis=1)
        relative[name] = {"t": data["t"], "p": relp, "v": relv, "radius": radius, "phase_error": phase_error,
                          "omega": omega, "position_error": error_position, "velocity_error": error_velocity,
                          "desired_radius": desired_radius, "desired_omega": desired_omega}
        metrics["agents"][name].update({"leader_join_missing": int((~valid).sum()),
            "formation_position_error_m": statistics(data["t"], error_position, tail_seconds, position_threshold),
            "formation_velocity_error_m_s": statistics(data["t"], error_velocity, tail_seconds, velocity_threshold),
            "radius_error_m": statistics(data["t"], radius-desired_radius, tail_seconds, position_threshold),
            "phase_error_rad": statistics(data["t"], phase_error, tail_seconds),
            "angular_rate_error_rad_s": statistics(data["t"], omega-desired_omega, tail_seconds)})
    fig, axes = plt.subplots(1, len(groups), figsize=(5*len(groups), 4.5), squeeze=False, constrained_layout=True)
    for i, group in enumerate(groups):
        radius = config["formation"]["radii"][i]
        angle = np.linspace(0, 2*np.pi, 201)
        axes[0, i].plot(radius*np.cos(angle), radius*np.sin(angle), "--", color="gray", label="desired circle")
        axes[0, i].plot(0, 0, "*", color="black", ms=10, label="UGV")
        for name in group["followers"]:
            if name in relative:
                axes[0, i].plot(*relative[name]["p"].T, label=name)
        axes[0, i].set(title="Group {}: relative to UGV".format(i+1), xlabel="Relative x (m)", ylabel="Relative y (m)")
        axes[0, i].set_aspect("equal", adjustable="box")
        axes[0, i].legend(fontsize=8)
    save(fig, "03_relative_orbits")
    fig, axes = plt.subplots(len(groups), 3, figsize=(13, 3*len(groups)), squeeze=False, constrained_layout=True)
    figerr, axerr = plt.subplots(len(groups), 2, figsize=(11, 3*len(groups)), squeeze=False, constrained_layout=True)
    for i, group in enumerate(groups):
        for name in group["followers"]:
            if name not in relative:
                continue
            data = relative[name]
            for column, key in enumerate(["radius", "phase_error", "omega"]):
                axes[i, column].plot(data["t"], data[key], label=name)
            axerr[i, 0].semilogy(data["t"], np.maximum(data["position_error"], 1e-12), label=name)
            axerr[i, 1].semilogy(data["t"], np.maximum(data["velocity_error"], 1e-12), label=name)
        axes[i, 0].axhline(config["formation"]["radii"][i], color="gray", ls="--")
        axes[i, 1].axhline(0, color="gray", ls="--")
        axes[i, 2].axhline(config["formation"]["angular_rates"][i], color="gray", ls="--")
        for column, title, unit in [(0, "Actual orbit radius", "m"), (1, "Wrapped phase error", "rad"), (2, "Relative orbit angular rate", "rad/s")]:
            axes[i, column].set(title="Group {}: {}".format(i+1, title), xlabel="Time from t0 (s)", ylabel=unit)
            axes[i, column].legend(fontsize=8)
        for column, title, unit, threshold in [(0, "Position error", "m", position_threshold), (1, "Velocity error", "m/s", velocity_threshold)]:
            axerr[i, column].set(title="Group {}: {}".format(i+1, title), xlabel="Time from t0 (s)", ylabel=unit)
            axerr[i, column].axhline(threshold, color="gray", ls="--")
            axerr[i, column].legend(fontsize=8)
    save(fig, "04_radius_phase_angular_rate")
    save(figerr, "05_formation_errors")

    fig, axes = plt.subplots(len(groups), 2, figsize=(12, 3*len(groups)), squeeze=False, constrained_layout=True)
    for name, data in arrays.items():
        i, follower = mapping[name]
        keys = ("alpha", "alpha_hat") if follower < 0 else ("beta", "beta_hat")
        for key, ls in zip(keys, ["-", "--"]):
            try:
                values = np.asarray([c.get(key, np.nan) for c in data["controller"]], dtype=float)
                if values.ndim == 1:
                    values = values[:, None]
                for j in range(values.shape[1]):
                    axes[i, 0 if follower < 0 else 1].plot(data["t"], values[:, j], ls,
                          label="{} {}{}".format(name, key, "[{}]".format(j) if values.shape[1] > 1 else ""), lw=1)
            except (TypeError, ValueError):
                pass
    for i in range(len(groups)):
        for j, title in enumerate(["Leader alpha / filtered alpha", "Follower beta / filtered beta"]):
            axes[i, j].set(title="Group {}: {}".format(i+1, title), xlabel="Time from t0 (s)", ylabel="Gain")
            axes[i, j].legend(fontsize=6, ncol=2)
    save(fig, "06_adaptive_gains")

    fig, axes = plt.subplots(len(groups), 2, figsize=(12, 3*len(groups)), squeeze=False, constrained_layout=True)
    figacc, axacc = plt.subplots(len(groups), 2, figsize=(12, 3*len(groups)), squeeze=False, constrained_layout=True)
    for name, data in arrays.items():
        i, follower = mapping[name]
        column = 0 if follower < 0 else 1
        axes[i, column].plot(data["t"], np.linalg.norm(data["raw"][:, :2], axis=1), label=name + " raw", lw=1)
        axes[i, column].plot(data["t"], np.linalg.norm(data["applied"][:, :2], axis=1), "--", label=name + " applied", lw=1)
        limited = data["limited"]
        if limited.any():
            axes[i, column].scatter(data["t"][limited], np.linalg.norm(data["applied"][limited, :2], axis=1), s=6, marker="x", color="black")
        axacc[i, 0].plot(data["t"], np.linalg.norm(data["accel"][:, :2], axis=1), label=name + " estimated", lw=1)
        if follower >= 0:
            axacc[i, 0].plot(data["t"], np.linalg.norm(data["applied"][:, :2], axis=1), "--", label=name + " command", lw=1)
        axacc[i, 1].plot(data["t"], np.linalg.norm(data["imu"], axis=1), label=name + " raw IMU", lw=1)
    for i in range(len(groups)):
        for j, title, unit in [(0, "UGV velocity commands", "m/s"), (1, "UAV acceleration commands", "m/s²")]:
            axes[i, j].set(title="Group {}: {}".format(i+1, title), xlabel="Time from t0 (s)", ylabel=unit)
            axes[i, j].legend(fontsize=6, ncol=2)
        axacc[i, 0].set(title="Group {}: world horizontal kinematic acceleration".format(i+1), xlabel="Time from t0 (s)", ylabel="m/s²")
        axacc[i, 1].set(title="Group {}: body IMU specific-force norm".format(i+1), xlabel="Time from t0 (s)", ylabel="m/s²")
        for ax in axacc[i]:
            ax.legend(fontsize=6, ncol=2)
        available_imu = any(np.isfinite(arrays[name]["imu"]).any() for name in [groups[i]["leader"]] + groups[i]["followers"] if name in arrays)
        if not available_imu:
            axacc[i, 1].legend().remove()
            axacc[i, 1].text(.5, .5, "No valid raw IMU samples recorded", transform=axacc[i, 1].transAxes,
                              ha="center", va="center", color="gray")
            axacc[i, 1].set_xlim(0, max(data["t"][-1] for data in arrays.values()))
    save(fig, "07_inputs_and_limits")
    save(figacc, "08_acceleration_and_raw_imu")

    fig, axes = plt.subplots(2, 2, figsize=(13, 7.5), constrained_layout=True)
    for name, data in arrays.items():
        axes[0, 0].plot(data["t"][1:], np.diff(data["stamp"]), label=name, lw=1)
    names = list(arrays)
    offsets = [metrics["agents"][name]["start_offset_s"] for name in names]
    axes[0, 1].bar(names, [offset if offset is not None else np.nan for offset in offsets])
    axes[0, 1].scatter(names, [offset if offset is not None else np.nan for offset in offsets], color="black", s=15, zorder=3)
    axes[0, 1].tick_params(axis="x", rotation=45)
    phase_names = sorted(phases)
    phase_index = {name: i for i, name in enumerate(phase_names)}
    by_stamp = sorted(all_records, key=lambda r: r["stamp"])
    axes[1, 0].scatter([r["stamp"]-t0 for r in by_stamp], [phase_index.get(r.get("phase", "UNKNOWN"), -1) for r in by_stamp], s=2)
    axes[1, 0].set_yticks(list(range(len(phase_names))))
    axes[1, 0].set_yticklabels(phase_names)
    axes[1, 1].bar(names, [metrics["agents"][name]["limited_fraction"] for name in names])
    axes[1, 1].scatter(names, [metrics["agents"][name]["limited_fraction"] for name in names], color="black", s=15, zorder=3)
    axes[1, 1].tick_params(axis="x", rotation=45)
    axes[0, 0].set(title="Observed telemetry sample intervals", xlabel="Time from t0 (s)", ylabel="Interval (s)")
    axes[0, 0].legend(ncol=3, fontsize=7)
    interval_series = [np.diff(data["stamp"]) for data in arrays.values() if len(data["stamp"]) > 1]
    intervals = np.concatenate(interval_series) if interval_series else np.array([])
    axes[0, 0].set_ylim(0, max(.01, float(intervals.max()) if len(intervals) else .01)*1.15)
    axes[0, 0].ticklabel_format(axis="y", style="plain", useOffset=False)
    axes[0, 1].set(title="Reported actual first-control startup offsets", ylabel="Offset from scheduled t0 (s)")
    axes[1, 0].set(title="Recorded phases including stop tail", xlabel="Time from t0 (s)")
    axes[1, 1].set(title="Fraction of formal samples with limited output", ylabel="Fraction", ylim=(0, 1))
    save(fig, "09_timing_and_data_quality")

    # Two-UAV separation is evaluated by matched measurements, not by 2R alone.
    fig, axes = plt.subplots(len(groups), 1, figsize=(10, 2.8*len(groups)), squeeze=False, constrained_layout=True)
    for i, group in enumerate(groups):
        first, second = [arrays.get(name) for name in group["followers"]]
        if first is None or second is None:
            continue
        matches = nearest_indices(first["stamp"], second["stamp"], max_gap)
        distance = np.full(len(matches), np.nan)
        valid = matches >= 0
        distance[valid] = np.linalg.norm(first["p"][valid] - second["p"][matches[valid]], axis=1)
        axes[i, 0].plot(first["t"], distance, label="Measured UAV separation")
        axes[i, 0].axhline(2*config["formation"]["radii"][i], ls="--", color="gray", label="Desired horizontal separation")
        axes[i, 0].set(title="Group {}: pairwise separation".format(i+1), xlabel="Time from t0 (s)", ylabel="Distance (m)")
        axes[i, 0].legend(fontsize=8)
        metrics.setdefault("group_uav_separation", {})[str(i)] = {"min_3d_distance_m": float(np.nanmin(distance)) if np.isfinite(distance).any() else None,
                                                                 "missing_matches": int((~valid).sum())}
    save(fig, "10_pairwise_uav_distance")

    fig, axes = plt.subplots(2, 2, figsize=(11, 10), constrained_layout=True)
    end = max(data["t"][-1] for data in arrays.values())
    snapshot_times = [0, min(10, end/3), min(30, 2*end/3), end]
    for ax, tau in zip(axes.flat, snapshot_times):
        for name, data in arrays.items():
            matched = nearest_indices([tau], data["t"], max_gap)[0]
            if matched >= 0:
                i, follower = mapping[name]
                ax.scatter(*data["p"][matched, :2], marker="*" if follower < 0 else "^", s=90 if follower < 0 else 35,
                           color=colors[i % len(colors)])
                ax.annotate(name, data["p"][matched, :2], xytext=(3, 3), textcoords="offset points", fontsize=7)
        ax.scatter(equilibrium[:, 0], equilibrium[:, 1], marker="+", color="black", s=60)
        ax.set(title="Measured formation at tau = {:.1f} s".format(tau), xlabel="World x (m)", ylabel="World y (m)")
        ax.set_aspect("equal", adjustable="box")
        all_xy = np.concatenate([data["p"][:, :2] for data in arrays.values()])
        if np.isfinite(all_xy).all():
            ax.set_xlim(all_xy[:, 0].min()-.2, all_xy[:, 0].max()+.2)
            ax.set_ylim(all_xy[:, 1].min()-.2, all_xy[:, 1].max()+.2)
    save(fig, "11_formation_snapshots")

    fig, axes = plt.subplots(len(groups), 2, figsize=(12, 3*len(groups)), squeeze=False, constrained_layout=True)
    figgap, axgap = plt.subplots(len(groups), 2, figsize=(12, 3*len(groups)), squeeze=False, constrained_layout=True)
    for name, data in arrays.items():
        i, follower = mapping[name]
        axes[i, 0].plot(data["t"], np.linalg.norm(data["v"][:, :2], axis=1), label=name, lw=1)
        axes[i, 1].plot(data["t"], data["p"][:, 2], label=name, lw=1)
        gain = "alpha" if follower < 0 else "beta"
        try:
            difference = np.asarray([c.get(gain, np.nan) for c in data["controller"]], dtype=float) - np.asarray(
                [c.get(gain+"_hat", np.nan) for c in data["controller"]], dtype=float)
            if difference.ndim == 1:
                difference = difference[:, None]
            for j in range(difference.shape[1]):
                axgap[i, 0 if follower < 0 else 1].semilogy(data["t"], np.maximum(np.abs(difference[:, j]), 1e-12),
                    label="{} {} gap{}".format(name, gain, "[{}]".format(j) if difference.shape[1] > 1 else ""), lw=1)
        except (TypeError, ValueError):
            pass
    for i in range(len(groups)):
        for j, title, unit in [(0, "Measured horizontal speed", "m/s"), (1, "Measured altitude", "m")]:
            axes[i, j].set(title="Group {}: {}".format(i+1, title), xlabel="Time from t0 (s)", ylabel=unit)
            axes[i, j].legend(fontsize=8)
        for j, title in enumerate(["Leader alpha - filtered alpha", "Follower beta - filtered beta"]):
            axgap[i, j].set(title="Group {}: {}".format(i+1, title), xlabel="Time from t0 (s)", ylabel="Absolute gain difference")
            axgap[i, j].legend(fontsize=7)
    save(fig, "12_velocity_and_altitude")
    save(figgap, "13_adaptive_gain_differences")
    metrics["figures"] = figures
    metrics["interpretation"] = ("Only RUNNING samples with nonnegative tau enter tracking metrics. "
        "Unmatched peer samples are missing; no zero filling. Recorded IMU body specific force is distinct from world "
        "kinematic acceleration and commanded acceleration. Orbit angular rate uses relative position and velocity. "
        "Settling times are sample-based and omitted for series with missing values.")
    (output / "metrics.json").write_text(json.dumps(clean_json(metrics), indent=2, ensure_ascii=False, allow_nan=False)+"\n", encoding="utf-8")
    (output / "analysis_config.json").write_text(json.dumps({"max_gap": max_gap, "tail_seconds": tail_seconds,
        "position_threshold": position_threshold, "velocity_threshold": velocity_threshold}, indent=2)+"\n", encoding="utf-8")
    lines = ["# 实验数据分析", "", "实验目录：`{}`".format(sid), "",
             "仅统计 RUNNING 状态且 tau >= 0 的正式控制段；停止尾段仅用于时序和数据质量展示。",
             "UAV 圆周误差使用相对本组 UGV 的实际位置与实际速度计算；缺失或超时匹配不填零。", "",
             "| 智能体 | 正式样本 | NE 或编队位置末段最大误差 (m) | 编队速度末段最大误差 (m/s) | 限幅比例 |",
             "|---|---:|---:|---:|---:|"]
    def fmt(value):
        return "--" if value is None else "{:.6g}".format(value)
    for name in ids:
        m = metrics["agents"].get(name, {})
        p = m.get("ne_error_m", m.get("formation_position_error_m", {})).get("tail_max_abs")
        v = m.get("formation_velocity_error_m_s", {}).get("tail_max_abs")
        lines.append("| {} | {} | {} | {} | {} |".format(name, m.get("formal_samples", 0), fmt(p), fmt(v), fmt(m.get("limited_fraction"))))
    lines += ["", "损坏 JSON 行：{}；缺少智能体：{}。".format(malformed, ", ".join(metrics["data_quality"]["missing_agents"]) or "无"),
              "", "记录的启动偏差是各控制节点的首次运行时间差，不等于设备执行的物理同步误差。",
              "运动学加速度估计、机体系 IMU 比力及命令加速度分别绘制。缺失加速度表示为无效值，不能解释为零加速度。",
              "对数图的 1e-12 下限仅用于显示，不改变统计数据。NE 参考是由保存配置离线求解，不进入在线控制器。",
              "本分析使用的位置、速度误差阈值可由命令行修改，应根据实物定位和测速精度设定。"]
    (output / "summary.md").write_text("\n".join(lines)+"\n", encoding="utf-8")
    return output, metrics


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("session_dir", type=Path)
    parser.add_argument("--picture-root", type=Path)
    parser.add_argument("--max-gap", type=float, default=.1)
    parser.add_argument("--tail-seconds", type=float, default=10)
    parser.add_argument("--position-threshold", type=float, default=.05)
    parser.add_argument("--velocity-threshold", type=float, default=.05)
    args = parser.parse_args()
    if min(args.max_gap, args.tail_seconds, args.position_threshold, args.velocity_threshold) <= 0:
        parser.error("All analysis tolerances must be positive")
    output, _ = analyze(args.session_dir, args.picture_root, args.max_gap, args.tail_seconds,
                        args.position_threshold, args.velocity_threshold)
    print("Analysis output: {}".format(output))


if __name__ == "__main__":
    main()
