from __future__ import annotations

import time

from typing import Optional

from app.sky_anchor_server import SkyAnchorServer
from app.vision import ShiftCommand


class CommandPublisher:
    """Publish shift commands to connected clients."""

    def __init__(
        self,
        host: str = "localhost",
        port: int = 8888,
        server: Optional[SkyAnchorServer] = None,
    ) -> None:
        self._server = server or SkyAnchorServer(host, port)
        self._server.start()

    def publish(self, command: ShiftCommand) -> None:
        payload = command.to_payload()
        payload["timestamp"] = time.time()
        self._server.tick(payload)

    def client_count(self) -> int:
        return self._server.get_client_count()

    def stop(self) -> None:
        self._server.stop()
