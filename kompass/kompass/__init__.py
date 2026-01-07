"""Kompass navigation stack"""

import sys
from importlib.metadata import version, PackageNotFoundError

# Minimum required version of kompass-core
MIN_KOMPASS_CORE_VERSION = "0.7.0"


def _parse_version(v_str):
    """
    Parses a version string into a tuple of integers for comparison.
    Handles standard 'major.minor.patch' formats.
    """
    try:
        # Split by '.' and take only integer parts (ignoring dev/rc tags for basic check)
        return tuple(int(part) for part in v_str.split(".") if part.isdigit())
    except ValueError:
        return (0, 0, 0)


# check for kompass_core installation
try:
    import kompass_core
except ImportError:
    print(
        "Welcome to Kompass! The fastest and most intuitive navigation stack known to man! "
        "Kompass uses the `kompass-core` python package, which implements highly parallelized navigation algorithms.\n"
        "Please install kompass-core using one of the following methods:\n"
        " - Install kompass-core with GPU support (Recommended):\n"
        "   `curl -sSL https://raw.githubusercontent.com/automatika-robotics/kompass-core/refs/heads/main/build_dependencies/install_gpu.sh | bash`\n"
        " - Install kompass-core from pypi without GPU support:\n"
        "   `pip install kompass-core`"
    )
    sys.exit(1)

# check for correct version
try:
    installed_ver_str = version("kompass-core")

    if _parse_version(installed_ver_str) < _parse_version(MIN_KOMPASS_CORE_VERSION):
        print(
            f"Error: Kompass requires `kompass-core` version {MIN_KOMPASS_CORE_VERSION} or higher.\n"
            f"You currently have version {installed_ver_str} installed.\n\n"
            "Please upgrade kompass-core using:\n"
            " - Install kompass-core with GPU support (Recommended):\n"
            "   `curl -sSL https://raw.githubusercontent.com/automatika-robotics/kompass-core/refs/heads/main/build_dependencies/install_gpu.sh | bash`\n"
            " - Install kompass-core from pypi without GPU support:\n"
            "   `pip install --upgrade kompass-core`"
        )
        sys.exit(1)

except PackageNotFoundError:
    # Fallback if package metadata is missing but import succeeded
    print("Warning: Could not determine `kompass-core` version. Proceeding with caution.")

__all__ = ["kompass_core"]
