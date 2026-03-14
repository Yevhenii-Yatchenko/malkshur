#!/usr/bin/env python3
"""
Sweep a Gazebo model across a grid, capture camera frames, and report ORB match percentages.

The script:
- Moves the specified model in a boustrophedon (zig-zag) pattern across a rectangular grid.
- After each 1m (default) move, grabs a frame from the configured camera (USB/CSI/Gazebo bridge).
- Compares the current frame to the previous one using ORB features and prints match percentage.
- At the end, prints the 10 positions with the lowest match percentages.

Example:
    python gazebo/grid_sweep_orb.py --model iris_demo --xmin -5 --xmax 5 --ymin -5 --ymax 5 --step 1 --z 2
"""

from __future__ import annotations

import argparse
import asyncio
import math
import os
import sys
import time
from pathlib import Path
from typing import List, Optional, Tuple

import cv2
import numpy as np
import pygazebo
from pygazebo.msg import model_pb2, pose_pb2

# Ensure repo paths are importable when run as a script
REPO_ROOT = Path(__file__).resolve().parent.parent
SKY_ANCHOR_ROOT = REPO_ROOT / "sky_anchor"
if str(SKY_ANCHOR_ROOT) not in sys.path:
    sys.path.append(str(SKY_ANCHOR_ROOT))
if str(REPO_ROOT) not in sys.path:
    sys.path.append(str(REPO_ROOT))

from app.frame.camera import CameraInitializer
from app.frame.normalization import normalize_image
from app.vision.parser import ParsedImage, get_image_parser
from src.logger import get_logger


def yaw_to_quaternion(yaw: float) -> Tuple[float, float, float, float]:
    """Convert yaw angle (radians) to quaternion components."""
    half = yaw * 0.5
    return 0.0, 0.0, math.sin(half), math.cos(half)


class GazeboModelMover:
    """Publish pose updates for a Gazebo model."""

    def __init__(
        self,
        host: str,
        port: int,
        model_name: str,
        loop: asyncio.AbstractEventLoop,
        pose_target: Optional[str] = None,
        override_topic: Optional[str] = None,
        override_name: Optional[str] = None,
    ) -> None:
        self.host = host
        self.port = port
        self.model_name = model_name
        self.loop = loop
        # pose_target allows publishing to a specific entity (e.g., model::link) on /pose/modify
        self.pose_target = pose_target or model_name
        self.override_topic = override_topic
        # override_name is the "name" field for PoseOverridePlugin messages (defaults to model_name)
        self.override_name = override_name or model_name
        self._manager = None
        self._publisher_model = None
        self._publisher_pose = None
        self._publisher_override = None

    async def connect(self) -> None:
        self._manager = await pygazebo.connect((self.host, self.port))
        # Model modify (primary)
        self._publisher_model = await self._manager.advertise('/gazebo/default/model/modify', 'gazebo.msgs.Model')
        try:
            await asyncio.wait_for(self._publisher_model.wait_for_listener(), timeout=5.0)
        except asyncio.TimeoutError:
            print("Warning: no listener on /gazebo/default/model/modify (will still publish)")
        # Pose modify (fallback)
        self._publisher_pose = await self._manager.advertise('/gazebo/default/pose/modify', 'gazebo.msgs.Pose')
        try:
            await asyncio.wait_for(self._publisher_pose.wait_for_listener(), timeout=5.0)
        except asyncio.TimeoutError:
            print("Warning: no listener on /gazebo/default/pose/modify (will still publish)")
        # Custom override topic (for PoseOverridePlugin)
        if self.override_topic:
            self._publisher_override = await self._manager.advertise(self.override_topic, 'gazebo.msgs.Pose')
            try:
                await asyncio.wait_for(self._publisher_override.wait_for_listener(), timeout=5.0)
            except asyncio.TimeoutError:
                print(f"Warning: no listener on {self.override_topic} (will still publish)")

    async def move_to(self, x: float, y: float, z: float, yaw: float) -> None:
        if self._publisher_model is None:
            raise RuntimeError("GazeboModelMover not connected")

        qx, qy, qz, qw = yaw_to_quaternion(yaw)
        msg = model_pb2.Model()
        msg.name = self.model_name
        msg.pose.position.x = x
        msg.pose.position.y = y
        msg.pose.position.z = z
        msg.pose.orientation.x = qx
        msg.pose.orientation.y = qy
        msg.pose.orientation.z = qz
        msg.pose.orientation.w = qw
        await self._publisher_model.publish(msg)

        # Also publish a plain pose update (some worlds respond to this for links)
        pose_msg = pose_pb2.Pose()
        pose_msg.name = self.pose_target
        pose_msg.position.x = x
        pose_msg.position.y = y
        pose_msg.position.z = z
        pose_msg.orientation.x = qx
        pose_msg.orientation.y = qy
        pose_msg.orientation.z = qz
        pose_msg.orientation.w = qw
        try:
            await self._publisher_pose.publish(pose_msg)
        except Exception:
            pass

        # Publish to override plugin if configured
        if self._publisher_override:
            try:
                override_msg = pose_pb2.Pose()
                override_msg.name = self.override_name
                override_msg.position.x = x
                override_msg.position.y = y
                override_msg.position.z = z
                override_msg.orientation.x = qx
                override_msg.orientation.y = qy
                override_msg.orientation.z = qz
                override_msg.orientation.w = qw
                await self._publisher_override.publish(override_msg)
            except Exception:
                pass

    def move(self, x: float, y: float, z: float, yaw: float) -> None:
        """Synchronous wrapper for convenience."""
        self.loop.run_until_complete(self.move_to(x, y, z, yaw))


def frange(start: float, stop: float, step: float):
    val = start
    epsilon = step * 0.5
    while val <= stop + epsilon:
        yield round(val, 6)
        val += step


def build_grid(xmin: float, xmax: float, ymin: float, ymax: float, step: float, z: float, yaw: float) -> List[Tuple[float, float, float, float]]:
    positions: List[Tuple[float, float, float, float]] = []
    y_values = list(frange(ymin, ymax, step))
    for row, y in enumerate(y_values):
        xs = list(frange(xmin, xmax, step))
        if row % 2 == 1:
            xs.reverse()  # zig-zag to minimize long jumps
        for x in xs:
            positions.append((x, y, z, yaw))
    return positions


def parse_args():
    parser = argparse.ArgumentParser(description="Sweep Gazebo model and compute ORB match percentages.")
    parser.add_argument("--host", default=os.environ.get("GAZEBO_HOST", "localhost"), help="Gazebo master host")
    parser.add_argument("--port", type=int, default=int(os.environ.get("GAZEBO_PORT", 11345)), help="Gazebo master port")
    parser.add_argument("--model", default="iris_demo", help="Model name to move (/model/modify)")
    parser.add_argument("--pose-target", default=None, help="Entity name for /pose/modify (e.g., model::link); defaults to --model")
    parser.add_argument("--override-topic", default="/gazebo/default/pose_override", help="Topic for PoseOverridePlugin (gazebo.msgs.Pose). Empty to disable.")
    parser.add_argument("--override-name", default=None, help="Name field sent to PoseOverridePlugin (defaults to --model)")
    parser.add_argument("--xmin", type=float, required=True, help="Grid min X")
    parser.add_argument("--xmax", type=float, required=True, help="Grid max X")
    parser.add_argument("--ymin", type=float, required=True, help="Grid min Y")
    parser.add_argument("--ymax", type=float, required=True, help="Grid max Y")
    parser.add_argument("--step", type=float, default=1.0, help="Step size (meters)")
    parser.add_argument("--z", type=float, default=2.0, help="Altitude (meters)")
    parser.add_argument("--yaw", type=float, default=0.0, help="Yaw (radians)")
    parser.add_argument("--settle", type=float, default=0.5, help="Seconds to wait after each move before capture")
    parser.add_argument("--log-level", default="INFO", help="Logger level (DEBUG/INFO/...)")
    return parser.parse_args()


def ensure_camera(logger):
    camera = CameraInitializer(logger).get_camera()
    if camera is None or not camera.isOpened():
        raise RuntimeError("Failed to initialize camera (check DRONE_CAMERA_TYPE/GAZEBO bridge).")
    return camera


def capture_parsed_frame(camera, parser, metadata=None) -> Optional[ParsedImage]:
    success, frame = camera.read()
    if not success or frame is None:
        return None
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    gray = normalize_image(gray)
    return parser.parse(gray, metadata=metadata)


def compute_match_percent(prev_img: ParsedImage, cur_img: ParsedImage, matcher) -> Tuple[float, int]:
    if prev_img.descriptors is None or cur_img.descriptors is None:
        return 0.0, 0
    matches = matcher.match(prev_img.descriptors, cur_img.descriptors)
    percent = (len(matches) / max(len(prev_img.keypoints), 1)) * 100.0
    return percent, len(matches)


def main():
    args = parse_args()
    logger = get_logger("grid_sweep", "logs/grid_sweep.log", log_level=args.log_level, console_output=True)

    loop = asyncio.new_event_loop()
    asyncio.set_event_loop(loop)

    logger.info(
        f"Starting grid sweep: model='{args.model}' host={args.host}:{args.port} "
        f"grid=({args.xmin},{args.xmax})x({args.ymin},{args.ymax}) step={args.step} z={args.z} yaw={args.yaw}"
    )

    mover = GazeboModelMover(
        args.host,
        args.port,
        args.model,
        loop,
        pose_target=args.pose_target,
        override_topic=args.override_topic or None,
        override_name=args.override_name or None,
    )
    logger.info("Connecting to Gazebo...")
    loop.run_until_complete(mover.connect())
    logger.info(
        f"Connected to Gazebo @ {args.host}:{args.port} for model '{args.model}' "
        f"(topics: /model/modify + /pose/modify target='{mover.pose_target}' override='{mover.override_topic}' override_name='{mover.override_name}')"
    )

    logger.info("Initializing camera and ORB parser...")
    camera = ensure_camera(logger)
    logger.info("Camera initialized.")
    parser = get_image_parser(logger)
    matcher = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=True)

    positions = build_grid(args.xmin, args.xmax, args.ymin, args.ymax, args.step, args.z, args.yaw)
    total = len(positions)
    logger.info(f"Total waypoints: {total}")

    previous_parsed: Optional[ParsedImage] = None
    previous_pos: Optional[Tuple[float, float, float, float]] = None
    stats = []

    for idx, (x, y, z, yaw) in enumerate(positions, start=1):
        logger.info(f"[{idx}/{total}] Moving '{args.model}' to ({x:.2f},{y:.2f},{z:.2f}) yaw={yaw:.2f}")
        mover.move(x, y, z, yaw)
        logger.debug(f"[{idx}/{total}] Move command published; waiting {args.settle:.2f}s for settle")
        time.sleep(args.settle)

        parsed = capture_parsed_frame(camera, parser, metadata={"idx": idx, "pos": (x, y, z)})
        if parsed is None:
            logger.warning(f"[{idx}/{total}] No frame captured at position ({x}, {y}, {z})")
            continue

        if previous_parsed is not None:
            percent, match_count = compute_match_percent(previous_parsed, parsed, matcher)
            stats.append({
                "percent": percent,
                "matches": match_count,
                "pos": (x, y, z),
                "prev_pos": previous_pos,
            })
            logger.info(f"[{idx}/{total}] Pose=({x:.2f},{y:.2f},{z:.2f}) ORB match vs prev: {percent:.2f}% ({match_count} matches)")
        else:
            logger.info(f"[{idx}/{total}] Pose=({x:.2f},{y:.2f},{z:.2f}) baseline frame captured.")

        previous_parsed = parsed
        previous_pos = (x, y, z)

    if stats:
        worst = sorted(stats, key=lambda s: s["percent"])[:10]
        logger.info("---- 10 lowest ORB match percentages ----")
        for rank, entry in enumerate(worst, start=1):
            px, py, pz = entry["pos"]
            logger.info(
                f"{rank:02d}: {entry['percent']:.2f}% matches={entry['matches']} at pos=({px:.2f},{py:.2f},{pz:.2f}) "
                f"vs prev={entry['prev_pos']}"
            )
    else:
        logger.warning("No match statistics collected (insufficient frames?).")


if __name__ == "__main__":
    main()
