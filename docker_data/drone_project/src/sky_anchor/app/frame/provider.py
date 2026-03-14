from __future__ import annotations

import threading
import time
from datetime import datetime
from typing import Any, Dict, Optional

import cv2
import numpy as np

from .camera import CameraInitializer
from ..config import FRAME_DEBUG_DIR, SLEEP_TIME
from ..logger import UnbufferedLogger
from .debug_writer import DebugImageWriter
from .normalization import normalize_image
from ..vision.parser import BaseImageParser, ParsedImage, get_image_parser


class FrameProvider:
    """Handle camera capture, normalization, and parsed image creation."""

    def __init__(
        self,
        logger: UnbufferedLogger,
        debug_dir: Optional[str] = None,
    ) -> None:
        self._logger = logger
        camera_init = CameraInitializer(logger)
        camera = camera_init.get_camera()
        if camera is None or not camera.isOpened():
            logger.error("Failed to initialize camera for frame provider")
            raise RuntimeError("Failed to initialize camera")
        self._camera = camera
        self._parser: BaseImageParser = get_image_parser(logger)
        debug_path = debug_dir or FRAME_DEBUG_DIR
        self._debug_writer = DebugImageWriter(logger, debug_dir=debug_path)
        self._frame_lock = threading.Lock()
        self._frame_ready = threading.Event()
        self._latest_frame: Optional[np.ndarray] = None
        self._capture_running = True
        self._capture_thread = threading.Thread(
            target=self._capture_loop,
            name="FrameCaptureThread",
            daemon=True,
        )
        self._capture_thread.start()

    def capture_current(self) -> Optional[ParsedImage]:
        """Capture the current frame."""

        capture_start = time.perf_counter()
        frame = self._get_frame(timeout=0.5)
        capture_read_end = time.perf_counter()
        if frame is None:
            return None

        parsed = self._build_parsed_frame(frame)
        capture_end = time.perf_counter()

        metadata = parsed.metadata or {}
        metadata.update({
            "capture_read_ms": (capture_read_end - capture_start) * 1000,
            "capture_process_ms": (capture_end - capture_read_end) * 1000,
        })
        parsed.metadata = metadata

        return parsed

    def _capture_loop(self) -> None:
        while self._capture_running:
            ret, frame = self._camera.read()
            if not ret or frame is None:
                self._logger.warning("Could not read frame from camera. Retrying...")
                time.sleep(SLEEP_TIME)
                continue
            with self._frame_lock:
                self._latest_frame = frame.copy()
                self._frame_ready.set()

    def _get_frame(self, timeout: float = 1.0) -> Optional[np.ndarray]:
        waited = self._frame_ready.wait(timeout)
        with self._frame_lock:
            frame = None if self._latest_frame is None else self._latest_frame.copy()
        if waited:
            self._frame_ready.clear()
        return frame

    def get_capture_stats(self) -> Optional[Dict[str, float]]:
        with self._frame_lock:
            latest = None if self._latest_frame is None else self._latest_frame
        if latest is None:
            return None
        return {
            "width": float(latest.shape[1]),
            "height": float(latest.shape[0]),
        }

    def _build_parsed_frame(
        self,
        frame: np.ndarray,
    ) -> ParsedImage:
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        gray = normalize_image(gray)
        capture_time = datetime.now()
        debug_file = self._debug_writer.save(gray, "frame", capture_time)

        metadata: Dict[str, Any] = {
            "captured_at": capture_time.isoformat(),
        }
        if debug_file:
            metadata["debug_file"] = debug_file

        return self._parser.parse(gray, metadata=metadata)

    def stop(self) -> None:
        self._capture_running = False
        if hasattr(self, "_capture_thread") and self._capture_thread.is_alive():
            self._capture_thread.join(timeout=1.0)
        if hasattr(self, "_camera"):
            try:
                self._camera.release()
            except Exception:
                pass

    def __del__(self) -> None:
        try:
            self.stop()
        except Exception:
            pass
