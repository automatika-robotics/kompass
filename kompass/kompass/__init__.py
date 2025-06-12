"""Kompass navigation stack"""

# check for kompass_core installation
try:
    import kompass_core
except ImportError:
    print(
        "Welcome to Kompass! The fastest and most intuitive navigation stack known to man! Kompass uses the `kompass-core` python package, which implements highly parallelized navigation algorithms. Please install kompass-core using one of the following methods:\n - Install kompass-core with GPU support (Recommended):\n  `curl https://raw.githubusercontent.com/automatika-robotics/kompass-core/refs/heads/main/build_dependencies/install_gpu.sh | bash`\n - Install kompass-core from pypi without GPU support:\n  `sudo apt-get install -y libompl-dev libfcl-dev libpcl-dev && pip install kompass-core`"
    )
    exit(1)
