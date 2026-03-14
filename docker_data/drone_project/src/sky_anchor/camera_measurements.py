#!/usr/bin/env python3
import sys
import time

from app.config import SLEEP_TIME
from app.frame.camera import CameraInitializer
from app.logger import get_logger


def main() -> None:
    logger = get_logger("camera_measure", log_level="INFO", console_output=True)
    camera = CameraInitializer(logger).get_camera()
    if camera is None or not camera.isOpened():
        logger.error("Failed to initialize camera")
        sys.exit(1)

    frame_idx = 0
    try:
        while True:
            start = time.perf_counter()
            ret, frame = camera.read()
            end = time.perf_counter()

            if not ret or frame is None:
                logger.warning(
                    "Could not read frame %s; retrying after %ss",
                    frame_idx,
                    SLEEP_TIME,
                )
                time.sleep(SLEEP_TIME)
                continue

            elapsed_ms = (end - start) * 1000.0
            print(f"{frame_idx}\t{elapsed_ms:.3f} ms\t{frame.shape[1]}x{frame.shape[0]}")
            frame_idx += 1
    finally:
        camera.release()


if __name__ == "__main__":
    main()
