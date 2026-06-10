"""Typed configuration value objects (GRASP Step 7, LC-2).

The dataclasses live in ``src.config.objects`` -- the one import path every
consumer uses (no re-exports here).  The tuning numbers themselves stay in
the existing ``src/altitude_config.py`` and ``src/position_config.py`` dict
modules; the dataclasses are constructed FROM those dicts (never
hand-copied), so the tuning remains byte-identical and single-sourced.
"""
