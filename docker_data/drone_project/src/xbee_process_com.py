"""Backward-compatibility shim: historical name, kept for Docker entrypoints.

The entry point now lives in run_controller.py; importing it starts the controller.
"""
from run_controller import *  # noqa: F401,F403
