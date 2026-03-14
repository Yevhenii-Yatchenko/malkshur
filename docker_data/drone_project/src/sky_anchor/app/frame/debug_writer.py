from __future__ import annotations

import os
from datetime import datetime
from typing import Optional

import cv2

from app.config import DEBUG_MODE
from app.logger import UnbufferedLogger


class DebugImageWriter:
    def __init__(self, logger: UnbufferedLogger, debug_dir: str = "debug_logs") -> None:
        self._logger = logger
        self._debug_dir = debug_dir

    def save(self, image, prefix: str, timestamp: datetime) -> Optional[str]:
        if not DEBUG_MODE:
            return None
        filename = self._filename(prefix, timestamp)
        self._ensure_dir()
        cv2.imwrite(os.path.join(self._debug_dir, filename), image)
        self._logger.info("Saved %s image as %s", prefix, filename)
        return filename

    def _ensure_dir(self) -> None:
        if not os.path.exists(self._debug_dir):
            os.makedirs(self._debug_dir)

    @staticmethod
    def _filename(prefix: str, timestamp: datetime) -> str:
        return f"{prefix}_{timestamp.strftime('%Y_%m_%d_%H:%M:%S.%f')}" + ".png"
