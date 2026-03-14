from __future__ import annotations

import math
from abc import ABC, abstractmethod
from typing import Iterable, Tuple

import cv2
import numpy as np

from ..config import ENABLE_CUDA
from .parser import ParsedImage


class ShiftEstimator(ABC):
    """Abstract base class for shift estimation using parsed images."""

    MIN_MATCH_PERCENT: int = 20

    def __init__(self, logger) -> None:
        self.logger = logger

    def estimate_shift(self, reference: ParsedImage, current: ParsedImage) -> Tuple[float, float, float, float]:
        """Estimate shift between two parsed images."""
        return self._compare_pair(reference, current)

    @abstractmethod
    def _compare(self, reference: ParsedImage, current: ParsedImage) -> Tuple[float, float, float, float]:
        """Subclass-specific comparison implementation."""

    def _ensure_descriptors(self, parsed: ParsedImage, context: str) -> None:
        if parsed.descriptors is None:
            label = parsed.metadata or context
            self.logger.error(f"No descriptors found in {context} image ({label}).")
            raise ValueError(f"No descriptors found in {context} image.")

    def _evaluate_matches(
        self,
        reference: ParsedImage,
        current: ParsedImage,
        matches: Iterable[cv2.DMatch],
    ) -> Tuple[float, float, float, float]:
        if not reference.keypoints:
            label = reference.metadata or "reference"
            self.logger.error(f"No keypoints available in reference image ({label}).")
            raise ValueError("Reference image has no keypoints.")

        match_percent = (len(matches) / len(reference.keypoints)) * 100
        if match_percent < self.MIN_MATCH_PERCENT:
            self.logger.error(f"Match percentage too low: {match_percent:.2f}%")
            raise ValueError(f"Match percentage too low: {match_percent:.2f}%")

        src_pts = np.float32([reference.keypoints[m.queryIdx].pt for m in matches]).reshape(-1, 1, 2)
        dst_pts = np.float32([current.keypoints[m.trainIdx].pt for m in matches]).reshape(-1, 1, 2)

        M, _ = cv2.findHomography(src_pts, dst_pts, cv2.RANSAC, 5.0)
        if M is None:
            self.logger.error("Homography computation failed.")
            raise ValueError("Homography computation failed.")

        dx = M[0, 2]
        dy = M[1, 2]
        rot_radians = math.atan2(M[1, 0], M[0, 0])
        angle_deg = math.degrees(rot_radians)

        return dx, dy, angle_deg, match_percent

    def _compare_pair(self, reference: ParsedImage, current: ParsedImage) -> Tuple[float, float, float, float]:
        self._ensure_descriptors(reference, "reference")
        self._ensure_descriptors(current, "current")
        self.logger.debug(
            "Comparing images ref=%s cur=%s",
            reference.metadata,
            current.metadata,
        )
        return self._compare(reference, current)


class CpuShiftEstimator(ShiftEstimator):
    """CPU-based ORB shift estimation."""

    def __init__(self, logger) -> None:
        super().__init__(logger)
        self.bf: cv2.BFMatcher = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=True)
        self.logger.info("Initialized CpuShiftEstimator")

    def _compare(self, reference: ParsedImage, current: ParsedImage) -> Tuple[float, float, float, float]:
        matches = sorted(
            self.bf.match(reference.descriptors, current.descriptors),
            key=lambda x: x.distance,
        )

        return self._evaluate_matches(reference, current, matches)


class CudaShiftEstimator(ShiftEstimator):
    """CUDA-accelerated ORB shift estimation."""

    def __init__(self, logger) -> None:
        super().__init__(logger)
        self.bf_cuda: cv2.DescriptorMatcher = cv2.cuda.DescriptorMatcher_createBFMatcher(cv2.NORM_HAMMING)
        self.logger.info("Initialized CudaShiftEstimator")

    def _ensure_gpu_descriptors(self, parsed: ParsedImage, context: str) -> None:
        if parsed.gpu_descriptors is None:
            label = parsed.metadata or context
            self.logger.error(f"No GPU descriptors found in {context} image ({label}).")
            raise ValueError(f"No descriptors found in {context} image.")

    def _compare(self, reference: ParsedImage, current: ParsedImage) -> Tuple[float, float, float, float]:
        self._ensure_gpu_descriptors(reference, "reference")
        self._ensure_gpu_descriptors(current, "current")

        matches_cuda = self.bf_cuda.match(reference.gpu_descriptors, current.gpu_descriptors)
        matches = sorted(matches_cuda, key=lambda x: x.distance)

        return self._evaluate_matches(reference, current, matches)


def get_shift_estimator(logger) -> ShiftEstimator:
    """Factory function to get the appropriate shift estimator based on configuration."""
    if ENABLE_CUDA:
        return CudaShiftEstimator(logger)
    return CpuShiftEstimator(logger)
