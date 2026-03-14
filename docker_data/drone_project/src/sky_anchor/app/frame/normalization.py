from __future__ import annotations

import cv2
import numpy as np

from app.config import NORMALIZE_TYPE


def normalize_image(
    image: np.ndarray,
    target_mean: float = 128,
) -> np.ndarray:
    if NORMALIZE_TYPE == 0:
        return image
    if NORMALIZE_TYPE == 1:
        return cv2.equalizeHist(image)
    if NORMALIZE_TYPE == 2:
        return _scale_to_mean(image, target_mean)
    raise ValueError(f"Unknown NORMALIZE_TYPE={NORMALIZE_TYPE}")


def _scale_to_mean(image: np.ndarray, target_mean: float) -> np.ndarray:
    mean = float(np.mean(image))
    adjustment_factor = target_mean / (mean + 1e-8)
    return np.clip(image * adjustment_factor, 0, 255).astype(np.uint8)
