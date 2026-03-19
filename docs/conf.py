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
myst_heading_anchors = 7  # to remove cross reference errors with md

html_theme = "shibuya"  # install with `pip install shibuya`
html_static_path = ["_static"]
html_css_files = [
    "custom.css",
]
html_favicon = "_static/favicon.png"

html_theme_options = {
    "announcement": 'Usage docs have moved to <a href="https://emos.automatikarobotics.com">EMOS Documentation</a>. This site only contains developer docs.',
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
        {"title": "EMOS Docs", "url": "https://emos.automatikarobotics.com/"},
        {"title": "Automatika Robotics", "url": "https://automatikarobotics.com/"},
    ],
}

# --- LLMS.TXT CONFIGURATION ---
# Developer documentation files for the LLM curriculum
LLMS_TXT_SELECTION = [
    # Developer Guide
    "development/adding_algorithms.md",
    "development/adding_python_algorithms.md",
    "development/adding_cpp_algorithms.md",
    "development/custom_component.md",
    "development/advanced_component.md",
    "development/custom_callbacks_publishers.md",
    "development/cli_reference.md",
    "development/architecture_references.md",
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
    preamble = """You are an expert robotics software engineer and developer assistant for **Kompass**, a high-performance, event-driven navigation stack built on ROS2 by Automatika Robotics.

You have been provided with the Kompass developer documentation. This covers the internal architecture, algorithm implementations, data type system, and extension patterns for contributors working on the framework.

For usage documentation including tutorials, installation guides, and configuration references, see [EMOS Documentation](https://emos.automatikarobotics.com).

The documentation is structured with file headers like `## File: filename.md`. Your primary task is to answer developer questions about extending the framework, explain internal architecture, and help with algorithm implementation based on this context.

Follow these rules rigorously:
1. **Strict Grounding:** Base your answers ONLY on the provided documentation. Do not hallucinate configuration parameters or API signatures that are not explicitly documented.
2. **Terminology Accuracy:** Use Kompass-specific terminology.
    - Refer to `Component` as the base execution unit (ROS 2 Lifecycle Node).
    - Reference `TopicsKeys` for I/O naming conventions.
    - When discussing GPU features, remember they are **vendor-neutral** (implemented via `kompass-core` and AdaptiveCpp/SYCL).
3. **Write Idiomatic Code:** Follow the patterns shown in the developer guide -- `attrs` `@define` for configs, `AllowedTopics` for I/O constraints, `ControlClasses`/`ControlConfigClasses` for algorithm registration.
4. **Handle Unknowns:** If a specific implementation detail is not in the text, state that it is not covered by the current documentation version.
5. **Cite Your Sources:** Briefly mention the file name when explaining topics.

Think step-by-step: Parse the developer's goal, locate the relevant architecture concepts, and synthesize a response that aligns with the Kompass patterns.\n\n"""

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


def setup(app):
    """Plugin to post build"""
    app.connect("build-finished", generate_llms_txt)
