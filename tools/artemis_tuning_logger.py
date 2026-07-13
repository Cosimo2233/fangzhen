from __future__ import annotations

import argparse
import json
import math
import pickle
import time
from pathlib import Path
from typing import Any

import zmq


DEFAULT_PARAMS: dict[str, float] = {
    "drive_velocity": 10.0,
    "track_velocity": 10.0,
    "turn_settle_s": 0.0,
    "entry_c_yaw_deg": -38.659808254090095,
    "entry_d_yaw_deg": -141.3401917459099,
    "yaw_kp": 0.3,
    "yaw_ki": 0.0,
    "yaw_kd": 0.015,
    "line_kp": 25.0,
    "line_ki": 0.0,
    "line_kd": 3.5,
}

POINTS = {
    "B": (1.6, 1.0),
    "C": (1.6, 0.2),
    "D": (0.6, 0.2),
}


def make_req(ctx: zmq.Context, endpoint: str) -> zmq.Socket:
    socket = ctx.socket(zmq.REQ)
    socket.setsockopt(zmq.RCVTIMEO, 2000)
    socket.setsockopt(zmq.SNDTIMEO, 2000)
    socket.setsockopt(zmq.LINGER, 0)
    socket.connect(endpoint)
    return socket


def health(ctx: zmq.Context, endpoint: str) -> dict[str, Any]:
    socket = make_req(ctx, endpoint)
    try:
        socket.send_string(json.dumps({"type": "health", "request_id": "logger-health"}))
        return json.loads(socket.recv_string())
    except Exception as exc:  # noqa: BLE001 - logger should keep running.
        return {"type": "error", "error": str(exc)}
    finally:
        socket.close(0)


def decode_viewer_frame(parts: list[bytes]) -> dict[str, Any]:
    payload = parts[-1]
    try:
        return json.loads(payload.decode("utf-8"))
    except Exception:
        return pickle.loads(payload)


def pose_from_message(message: dict[str, Any]) -> dict[str, Any]:
    pose = message.get("pose")
    kinematics = message.get("kinematics") if isinstance(message.get("kinematics"), dict) else {}
    if isinstance(pose, dict):
        x = float(pose.get("x_m", 0.0))
        y = float(pose.get("y_m", 0.0))
        yaw = float(pose.get("yaw_rad", 0.0))
        speed = float(kinematics.get("longitudinal_velocity_m_s", 0.0))
        yaw_rate = float(kinematics.get("yaw_rate_rad_s", 0.0))
    else:
        qpos = list(message.get("qpos", []))
        qvel = list(message.get("qvel", []))
        x = float(qpos[0]) if len(qpos) > 0 else 0.0
        y = float(qpos[1]) if len(qpos) > 1 else 0.0
        yaw = float(qpos[2]) if len(qpos) > 2 else 0.0
        speed = math.hypot(float(qvel[0]), float(qvel[1])) if len(qvel) >= 2 else 0.0
        yaw_rate = float(qvel[2]) if len(qvel) >= 3 else 0.0
    distances = {
        name: round(math.hypot(x - point[0], y - point[1]) * 100.0, 3)
        for name, point in POINTS.items()
    }
    return {
        "time": float(message.get("time", 0.0)),
        "sequence_id": int(message.get("sequence_id", 0)),
        "x_m": x,
        "y_m": y,
        "yaw_deg": math.degrees(yaw),
        "speed_m_s": speed,
        "yaw_rate_deg_s": math.degrees(yaw_rate),
        "distance_to_point_cm": distances,
    }


def latest_viewer_pose(ctx: zmq.Context, endpoint: str, wait_s: float) -> dict[str, Any] | None:
    socket = ctx.socket(zmq.SUB)
    socket.setsockopt_string(zmq.SUBSCRIBE, "")
    socket.setsockopt(zmq.RCVTIMEO, int(wait_s * 1000.0))
    socket.connect(endpoint)
    latest: dict[str, Any] | None = None
    end = time.monotonic() + wait_s
    try:
        while time.monotonic() < end:
            try:
                latest = pose_from_message(decode_viewer_frame(socket.recv_multipart()))
            except zmq.Again:
                break
    finally:
        socket.close(0)
    return latest


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--viewer", default="tcp://192.168.1.24:5555")
    parser.add_argument("--ai-command", default="tcp://127.0.0.1:5560")
    parser.add_argument("--output", required=True)
    parser.add_argument("--interval-s", type=float, default=30.0)
    args = parser.parse_args()

    output = Path(args.output)
    output.parent.mkdir(parents=True, exist_ok=True)
    ctx = zmq.Context.instance()

    while True:
        record = {
            "wall_time_s": time.time(),
            "params_source": "codex_known_commands",
            "params": DEFAULT_PARAMS,
            "bridge": health(ctx, args.ai_command),
            "viewer": latest_viewer_pose(ctx, args.viewer, wait_s=2.0),
        }
        with output.open("a", encoding="utf-8") as file:
            file.write(json.dumps(record, ensure_ascii=False, sort_keys=True) + "\n")
        print(json.dumps(record, ensure_ascii=False, sort_keys=True), flush=True)
        time.sleep(args.interval_s)


if __name__ == "__main__":
    raise SystemExit(main())
