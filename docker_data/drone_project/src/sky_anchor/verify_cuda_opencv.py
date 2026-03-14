#!/usr/bin/env python3
"""Verify that the OpenCV build exposes the CUDA features required by Sky Anchor.

The script checks:
  * OpenCV import success and version display.
  * Presence of the ``cv2.cuda`` module.
  * At least one CUDA-capable device detected by OpenCV.
  * Ability to construct ``cv2.cuda_ORB`` and run ``detectAndComputeAsync``.
  * Ability to create a CUDA brute-force matcher for Hamming distance.

It exits with code 0 if all checks pass, otherwise prints diagnostics and exits 1.
"""
from __future__ import annotations

import sys
from typing import Tuple


def main() -> int:
    try:
        import cv2
    except ImportError as exc:  # pragma: no cover - import failure path
        print(f"[FAIL] Unable to import OpenCV: {exc}", file=sys.stderr)
        return 1

    print(f"OpenCV version: {cv2.__version__}")

    if not hasattr(cv2, "cuda"):
        print("[FAIL] cv2.cuda module is not available. Install a CUDA-enabled OpenCV build.", file=sys.stderr)
        return 1

    try:
        device_count = cv2.cuda.getCudaEnabledDeviceCount()
    except cv2.error as exc:  # pragma: no cover - cuda query failure path
        print(f"[FAIL] Unable to query CUDA devices via OpenCV: {exc}", file=sys.stderr)
        return 1

    if device_count <= 0:
        print("[FAIL] OpenCV reports zero CUDA-enabled devices.", file=sys.stderr)
        return 1

    print(f"CUDA devices detected: {device_count}")

    if not hasattr(cv2.cuda, "ORB"):
        print("[FAIL] cv2.cuda_ORB is unavailable in this build.", file=sys.stderr)
        return 1

    try:
        orb = cv2.cuda_ORB.create()
        print("Created cv2.cuda_ORB instance.")
    except cv2.error as exc:  # pragma: no cover - orb creation failure path
        print(f"[FAIL] Unable to construct cv2.cuda_ORB: {exc}", file=sys.stderr)
        return 1

    # Construct a dummy grayscale image and ensure ORB produces descriptors.
    import numpy as np

    dummy_image = (np.random.rand(480, 640) * 255).astype(np.uint8)
    gpu_mat = cv2.cuda_GpuMat()
    gpu_mat.upload(dummy_image)

    try:
        keypoints_gpu, descriptors_gpu = orb.detectAndComputeAsync(gpu_mat, None)
        keypoints = orb.convert(keypoints_gpu)
        descriptors = descriptors_gpu.download() if descriptors_gpu is not None else None
    except cv2.error as exc:
        print(f"[FAIL] ORB detectAndComputeAsync failed: {exc}", file=sys.stderr)
        return 1

    kp_count = len(keypoints)
    desc_shape: Tuple[int, int] | str = descriptors.shape if descriptors is not None else "None"
    print(f"ORB keypoints: {kp_count}, descriptor shape: {desc_shape}")
    if kp_count == 0 or descriptors is None:
        print("[FAIL] ORB produced no keypoints/descriptors on a test image.", file=sys.stderr)
        return 1

    try:
        matcher = cv2.cuda.DescriptorMatcher_createBFMatcher(cv2.NORM_HAMMING)
        print("Created CUDA brute-force matcher (Hamming).")
        # Run a trivial match against itself to ensure kernels load.
        matches = matcher.match(descriptors_gpu, descriptors_gpu)
        print(f"Matcher produced {len(matches)} self-matches (expected >= keypoints).")
    except cv2.error as exc:
        print(f"[FAIL] Unable to create/use CUDA descriptor matcher: {exc}", file=sys.stderr)
        return 1

    print("All CUDA checks passed.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
