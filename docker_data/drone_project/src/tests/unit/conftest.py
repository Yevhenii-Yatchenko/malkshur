"""Shared configuration for the unit test suite.

Tests import production modules exactly like the live code does
(``from src.pid_controller import ...``), which requires the project root
(``/drone_project`` inside the container) to be on ``sys.path``.  Running
``python3 -m pytest`` from the project root already guarantees that; this
guard makes the suite robust to other invocation styles as well.

No production code is imported here and no files are written.
"""

import os
import sys

PROJECT_ROOT = os.path.abspath(
    os.path.join(os.path.dirname(__file__), os.pardir, os.pardir)
)

if PROJECT_ROOT not in sys.path:
    sys.path.insert(0, PROJECT_ROOT)


def pytest_configure(config):
    """Register the markers used by this suite.

    They mirror the ``markers`` section of the project's pytest.ini, but that
    file is currently inert: it uses a ``[tool:pytest]`` section header, which
    pytest only honors in setup.cfg -- in a pytest.ini it is ignored entirely
    (so its addopts/testpaths/markers never applied).  Registering here keeps
    the suite warning-clean without touching the (pre-existing) config defect.
    """
    config.addinivalue_line("markers", "unit: Unit tests for individual components")
    config.addinivalue_line(
        "markers", "pid: Tests related to PID controller functionality"
    )
    config.addinivalue_line(
        "markers", "communication: Tests related to XBee communication"
    )
