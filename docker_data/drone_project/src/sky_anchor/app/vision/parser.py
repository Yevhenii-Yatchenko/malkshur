from __future__ import annotations

from dataclasses import dataclass
from typing import Any, Dict, List, Optional, Tuple

import cv2
import numpy as np

from ..config import ENABLE_CUDA, ORB_NFEATURES


@dataclass
class ParsedImage:
    """Feature-rich representation of an image ready for shift comparison."""

    gray: np.ndarray
    keypoints: List[cv2.KeyPoint]
    descriptors: Optional[np.ndarray]
    metadata: Optional[Dict[str, Any]] = None
    gpu_image: Optional[Any] = None
    gpu_keypoints: Optional[Any] = None
    gpu_descriptors: Optional[Any] = None


class BaseImageParser:
    """Base parser capable of turning frames into parsed images."""

    def __init__(self, logger, nfeatures: int = ORB_NFEATURES) -> None:
        self.logger = logger
        self.nfeatures = nfeatures

    @staticmethod
    def _ensure_gray(image: np.ndarray) -> np.ndarray:
        if image.ndim == 2:
            return image
        return cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

    def parse(self, image: np.ndarray, metadata: Optional[Dict[str, Any]] = None) -> ParsedImage:
        gray = self._ensure_gray(image)
        keypoints, descriptors = self._compute_features(gray)
        return ParsedImage(gray=gray, keypoints=keypoints, descriptors=descriptors, metadata=metadata)

    def _compute_features(self, gray: np.ndarray) -> Tuple[List[cv2.KeyPoint], Optional[np.ndarray]]:
        raise NotImplementedError


class CpuImageParser(BaseImageParser):
    """CPU-backed ORB feature extractor."""

    def __init__(self, logger, nfeatures: int = ORB_NFEATURES) -> None:
        super().__init__(logger, nfeatures=nfeatures)
        self.logger.info("Initializing CPU ORB parser")
        self.orb = cv2.ORB_create(
            nfeatures=self.nfeatures,
            scaleFactor=1.2,
            nlevels=30,
        )

    def _compute_features(self, gray: np.ndarray) -> Tuple[List[cv2.KeyPoint], Optional[np.ndarray]]:
        keypoints, descriptors = self.orb.detectAndCompute(gray, None)
        return keypoints, descriptors


class CudaImageParser(BaseImageParser):
    """CUDA-backed ORB feature extractor."""

    def __init__(self, logger, nfeatures: int = ORB_NFEATURES) -> None:
        super().__init__(logger, nfeatures=nfeatures)
        if not hasattr(cv2, 'cuda'):
            raise RuntimeError("CUDA support not available in current OpenCV build")
        self.logger.info("Initializing CUDA ORB parser")
        self.cuda_orb = cv2.cuda_ORB.create(nfeatures=self.nfeatures)

    def parse(self, image: np.ndarray, metadata: Optional[Dict[str, Any]] = None) -> ParsedImage:
        gray = self._ensure_gray(image)
        gpu_image = cv2.cuda_GpuMat()
        gpu_image.upload(gray)

        keypoints_gpu, descriptors_gpu = self.cuda_orb.detectAndComputeAsync(gpu_image, None)
        keypoints_cpu = self.cuda_orb.convert(keypoints_gpu)
        descriptors_cpu = descriptors_gpu.download() if descriptors_gpu is not None else None

        return ParsedImage(
            gray=gray,
            keypoints=keypoints_cpu,
            descriptors=descriptors_cpu,
            metadata=metadata,
            gpu_image=gpu_image,
            gpu_keypoints=keypoints_gpu,
            gpu_descriptors=descriptors_gpu,
        )

    def _compute_features(self, gray: np.ndarray) -> Tuple[List[cv2.KeyPoint], Optional[np.ndarray]]:  # pragma: no cover - not used directly
        raise NotImplementedError("Use parse() for CUDA parser to retain GPU buffers")


def get_image_parser(logger: Any) -> BaseImageParser:
    """Return a parser appropriate for the configured backend."""
    if ENABLE_CUDA:
        try:
            return CudaImageParser(logger)
        except (RuntimeError, AttributeError, cv2.error) as exc:
            logger.warning("CUDA parser unavailable: %s. Falling back to CPU parser.", exc)
    return CpuImageParser(logger)
