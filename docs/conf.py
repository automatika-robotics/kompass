# Configuration file for the Sphinx documentation builder.
import os
import sys
from datetime import date
import xml.etree.ElementTree as ET
from pathlib import Path

# Flag to signal that we are building documentation.
# This prevents __init__.py from running runtime dependency checks.
os.environ["KOMPASS_DOCS_BUILD"] = "1"

sys.path.insert(0, os.path.abspath(".."))
version = ET.parse("../kompass/package.xml").getroot()[1].text
print("Found version:", version)

project = "Kompass"
copyright = f"{date.today().year}, Automatika Robotics"
author = "Automatika Robotics"
release = version

extensions = [
    "sphinx.ext.viewcode",
    "sphinx.ext.doctest",
    "sphinx_copybutton",  # install with `pip install sphinx-copybutton`
    "autodoc2",  # install with `pip install sphinx-autodoc2`
    "myst_parser",  # install with `pip install myst-parser`
    "sphinx_sitemap",  # install with `pip install sphinx-sitemap`
    "sphinx_design",  # install with `pip install sphinx-design`
]

autodoc2_packages = [
    {
        "path": "../kompass/kompass",
        "module": "kompass",
        "exclude_files": [
            "utils.py",
            "components/utils.py",
        ],
    },
]

autodoc2_module_all_regexes = [r"components\*"]

autodoc2_hidden_objects = ["private", "dunder", "undoc"]

autodoc2_class_docstring = "both"  # bug in autodoc2, should be `merge`

autodoc2_render_plugin = "myst"

templates_path = ["_templates"]
exclude_patterns = ["_build", "Thumbs.db", ".DS_Store", "README*"]

myst_enable_extensions = [
    "amsmath",
    "attrs_inline",
    "colon_fence",
    "deflist",
    "dollarmath",
    "fieldlist",
    "html_admonition",
    "html_image",
    "linkify",  # install with pip install linkify-it-py
    "replacements",
    "smartquotes",
    "strikethrough",
    "substitution",
    "tasklist",
]
language = "en"
myst_html_meta = {
    "google-site-verification": "cQVj-BaADcGVOGB7GOvfbkgJjxni10C2fYWCZ03jOeo"
}
myst_heading_anchors = 7  # to remove cross reference errors with md

html_baseurl = "https://kompass.automatikarobotics.com/"
html_theme = "shibuya"  # install with `pip install shibuya`
html_static_path = ["_static"]
html_css_files = [
    "custom.css",
]
html_favicon = "_static/favicon.png"
sitemap_url_scheme = "{link}"

html_theme_options = {
    "light_logo": "_static/Kompass_light.png",
    "dark_logo": "_static/Kompass_dark.png",
    "accent_color": "indigo",
    "twitter_url": "https://x.com/__automatika__",
    "github_url": "https://github.com/automatika-robotics/kompass",
    "discord_url": "https://discord.gg/B9ZU6qjzND",
    "globaltoc_expand_depth": 1,
    "open_in_chatgpt": True,
    "open_in_claude": True,
    # Navigation Links (Top bar)
    "nav_links": [
        {"title": "Automatika Robotics", "url": "https://automatikarobotics.com/"},
    ],
}

# --- LLMS.TXT CONFIGURATION ---
# Defines the order of manual documentation for the curriculum
LLMS_TXT_SELECTION = [
    # Introduction & Setup
    "overview.md",
    "why.md",
    "install.md",
    "cli.md",
    # Getting Started (Simulations & Basics)
    "tutorials/quick_start.md",
    "tutorials/quick_start_gazebo.md",
    "tutorials/quick_start_webots.md",
    "tutorials/configuration.md",
    # Core Architecture (Navigation Stack Definition)
    "navigation/robot.md",
    "navigation/driver.md",
    "navigation/control.md",
    "navigation/path_planning.md",
    "navigation/motion_server.md",
    "navigation/mapper.md",
    "navigation/map_server.md",
    "navigation/mapping_localization.md",
    # Key Capabilities & Tutorials
    "tutorials/point_navigation.md",
    "tutorials/events_actions.md",
    "tutorials/vision_tracking.md",
    "tutorials/vision_tracking_depth.md",
    "tutorials/record_load_path.md",
    "tutorials/events_composed.md",
    "tutorials/events_cross_healing.md",
    "tutorials/events_dynamic.md",
    "tutorials/events_external_reflexes.md",
    "tutorials/fallbacks_simple.md",
    "tutorials/automated_motion_test.md",
    # Advanced Concepts & System Design
    "advanced/design.md",
    "advanced/types.md",
    "advanced/extending.md",
    "advanced/advanced_conf/topics.md",
    "advanced/advanced_conf/qos.md",
    # Algorithms (Technical Details)
    "advanced/algorithms/dwa.md",
    "advanced/algorithms/pure_pursuit.md",
    "advanced/algorithms/stanley.md",
    "advanced/algorithms/vision_follower.md",
    "advanced/algorithms/dvz.md",
    "advanced/algorithms/cost_eval.md",
    # Integrations
    "integrations/ompl.md",
    "integrations/fcl.md",
]


def format_for_llm(filename: str, content: str) -> str:
    """Helper to wrap content in a readable format for LLMs."""
    # Clean up HTML image tags to reduce noise
    lines = content.split("\n")
    cleaned_lines = [line for line in lines if "<img src=" not in line]
    cleaned_content = "\n".join(cleaned_lines).strip()

    return f"## File: {filename}\n```markdown\n{cleaned_content}\n```\n\n"


def generate_llms_txt(app, exception):
    """Generates llms.txt combining manual docs and autodoc2 API docs."""
    if exception is not None:
        return  # Do not generate if build failed

    print("[llms.txt] Starting generation...")

    src_dir = Path(app.srcdir)
    out_dir = Path(app.outdir)
    full_text = []

    # Add Preamble
    # Add Preamble
    preamble = """You are an expert robotics software engineer and developer assistant for **Kompass**, a high-performance, event-driven navigation stack built on ROS2 by Automatika Robotics.

You have been provided with the official Kompass documentation. This framework distinguishes itself through **hardware-agnostic GPU acceleration** (supporting Nvidia, AMD, and integrated GPUs) and a an event-driven architecture with a simple API where you can build your entire application in one **Python Script 'Recipe'**.

The documentation is structured with file headers like `## File: filename.md`. Your primary task is to answer user questions, explain navigation concepts, and generate configuration or code strictly based on this context.

Follow these rules rigorously:
1. **Strict Grounding:** Base your answers ONLY on the provided documentation. Do not hallucinate configuration parameters (e.g., for DWA or Pure Pursuit) that are not explicitly listed in the `advanced/algorithms` or `configuration.md` files.
2. **Terminology Accuracy:** Use Kompass-specific terminology.
    - Refer to Python applications as **"Recipes"**.
    - When discussing GPU features, remember they are **vendor-neutral** (implemented via `kompass-core`).
3. **Write Idiomatic Code:**
    - For configuration: Follow the YAML structures shown in `tutorials/configuration.md`.
    - For scripts: Use the event-driven patterns (e.g., `events_actions.md`) and the standard `Robot` -> `Driver` -> `Control` hierarchy.
4. **Handle Unknowns:** If a specific algorithm implementation or parameter is not in the text, state that it is not covered by the current documentation version.
5. **Cite Your Sources:** Briefly mention the file name (e.g., "See `integrations/ompl.md`...") when explaining complex topics like path planning or collision checking.

Think step-by-step: Parse the user's goal, locate the relevant modules (e.g., Mapper vs. Planner), and synthesize a response that aligns with the Kompass philosophy of event-driven, modular navigation.\n\n"""

    full_text.append(preamble)

    # Process Manual Docs (Curated List)
    print(f"[llms.txt] Processing {len(LLMS_TXT_SELECTION)} manual files...")
    for relative_path in LLMS_TXT_SELECTION:
        file_path = src_dir / relative_path
        if file_path.exists():
            content = file_path.read_text(encoding="utf-8")
            full_text.append(format_for_llm(relative_path, content))
        else:
            print(f"[llms.txt] Warning: Manual file not found: {relative_path}")

    # Write output to the build root
    output_path = out_dir / "llms.txt"
    try:
        output_path.write_text("".join(full_text), encoding="utf-8")
        print(f"[llms.txt] Successfully generated: {output_path}")
    except Exception as e:
        print(f"[llms.txt] Error writing file: {e}")


def create_robots_txt(app, exception):
    """Create robots.txt file to take advantage of sitemap crawl"""
    if exception is None:
        dst_dir = app.outdir  # Typically 'build/html/'
        robots_path = os.path.join(dst_dir, "robots.txt")
        content = f"""User-agent: *

Sitemap: {html_baseurl}sitemap.xml
"""
        with open(robots_path, "w") as f:
            f.write(content)


def setup(app):
    """Plugin to post build"""
    app.connect("build-finished", create_robots_txt)
    app.connect("build-finished", generate_llms_txt)
