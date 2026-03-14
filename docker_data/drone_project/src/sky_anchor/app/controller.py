from __future__ import annotations

import time

from app.config import (
    SLEEP_TIME,
    SKY_ANCHOR_LOG_PATH,
)
from app.command_publisher import CommandPublisher
from app.frame import FrameProvider
from app.logger import UnbufferedLogger, get_logger
from app.navigation import NavigationCoordinator
from app.profiler import PipelineProfiler
from app.vision import ShiftEvaluator

from app.navigation.types import NavigationTarget


class Controller:

    def __init__(self) -> None:
        self.__logger: UnbufferedLogger = get_logger(log_file_path=SKY_ANCHOR_LOG_PATH)
        self.__frame_provider = FrameProvider(logger=self.__logger)
        self.__shift_evaluator = ShiftEvaluator(logger=self.__logger)
        self.__command_publisher = CommandPublisher()
        self.__profiler = PipelineProfiler(logger=self.__logger)
        self.__navigation = NavigationCoordinator(logger=self.__logger)

    def run(self) -> None:
        self.__logger.info("Starting real drone drift correction...")

        reference = None
        while reference is None:
            reference = self.__frame_provider.capture_current()
            if reference is None:
                self.__logger.error("Could not establish an initial reference frame. Exiting.")
            time.sleep(0.1)

        while True:
            cycle_start = time.perf_counter()
            current_image = self.__frame_provider.capture_current()
            capture_end = time.perf_counter()
            if current_image is None:
                continue

            try:
                command = self.__shift_evaluator.evaluate(reference, current_image)
                evaluate_end = time.perf_counter()
            except Exception as exc:
                self.__logger.error(f"Error computing shift: {exc}")
                # reference = current_image
                continue

            modified_command, should_update_reference = self.__navigation.process(command)

            if should_update_reference:
                reference = current_image

            command = modified_command

            self.__command_publisher.publish(command)
            publish_end = time.perf_counter()

            self.__profiler.record_cycle(
                capture_start=cycle_start,
                capture_end=capture_end,
                evaluate_end=evaluate_end,
                publish_end=publish_end,
            )
