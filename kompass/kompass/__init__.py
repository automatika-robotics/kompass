"""Kompass navigation stack"""

# check for kompass_core installation
try:
    import kompass_core
except ImportError:
    # TODO: Adds GPU install string
    print(
        "Welcome to Kompass! The fastest and most intuitive navigation stack known to man! Kompass uses the `kompass-core` python package, which implements highly paralellized navigation algorithms. Please install kompass-core using one of the following methods:\n - Install kompass-core with GPU support (Recommended):\n  `installation string`\n - Install kompass-core from pypi without GPU support:\n  `pip install kompass-core`"
    )
    exit(1)
