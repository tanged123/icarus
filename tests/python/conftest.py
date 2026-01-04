"""Pytest configuration for Icarus Python tests."""

import os
import sys

# Add build directory to path for finding the module
build_python_dir = os.path.join(os.path.dirname(__file__), "..", "..", "build", "python")
if os.path.exists(build_python_dir):
    sys.path.insert(0, build_python_dir)


def pytest_configure(config):
    """Configure pytest."""
    # Ensure we can import icarus
    try:
        import icarus

        print(f"Using icarus version {icarus.__version__}")
    except ImportError as e:
        raise ImportError(
            f"Could not import icarus module. "
            f"Make sure you've built with --python flag and the module is in {build_python_dir}. "
            f"Error: {e}"
        )
