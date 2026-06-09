"""Backward-compatibility shim: historical name, kept for Docker entrypoints.

The entry point now lives in run_controller.py; importing it starts the controller.
Importing run_controller from program code must never be done (the controller
starts at import and runs forever inside the import; a threaded import would
deadlock) — this file exists only for ``python3 xbee_process_com.py`` invocation
by Docker entrypoints.
"""
from run_controller import *  # noqa: F401,F403
