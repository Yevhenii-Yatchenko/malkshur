"""Entry point for the Sky Anchor application."""

from __future__ import annotations

from app.controller import Controller


def main() -> None:
    """Kick off the Sky Anchor controller loop."""

    controller = Controller()
    controller.run()


if __name__ == "__main__":
    main()
