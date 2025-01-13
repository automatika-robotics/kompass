# Configuration file for the Sphinx documentation builder.
import os
import sys
from datetime import date
import xml.etree.ElementTree as ET


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
exclude_patterns = ["_build", "Thumbs.db", ".DS_Store"]

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


html_theme = "sphinx_book_theme"  # install with `pip install sphinx-book-theme`
html_static_path = ["_static"]

html_theme_options = {
    "logo": {
        "image_light": "_static/Kompass_dark.png",
        "image_dark": "_static/Kompass.png",
    },
    "path_to_docs": "docs",
    "repository_url": "https://github.com/automatika-robotics/kompass",
    "repository_branch": "main",
    "use_source_button": True,
    "use_issues_button": True,
    "use_edit_page_button": True,
}
