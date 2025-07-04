# Configuration file for the Sphinx documentation builder.
import os
import sys
from datetime import date
import xml.etree.ElementTree as ET
import shutil


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

html_baseurl = "https://automatika-robotics.github.io/kompass/"
html_theme = "sphinx_book_theme"  # install with `pip install sphinx-book-theme`
html_static_path = ["_static"]
html_css_files = [
    "custom.css",
]
html_favicon = "_static/favicon.png"

html_theme_options = {
    "logo": {
        "image_light": "_static/Kompass_light.png",
        "image_dark": "_static/Kompass_dark.png",
    },
    "icon_links": [
        {
            "name": "Automatika",
            "url": "https://automatikarobotics.com/",
            "icon": "_static/automatika-logo.png",
            "type": "local",
        },
        {
            "name": "GitHub",
            "url": "https://github.com/automatika-robotics/kompass",
            "icon": "fa-brands fa-github",
        },
        {
            "name": "Discord",
            "url": "https://discord.gg/B9ZU6qjzND",
            "icon": "fa-brands fa-discord",
        },
    ],
    "path_to_docs": "docs",
    "repository_url": "https://github.com/automatika-robotics/kompass",
    "repository_branch": "main",
    "use_source_button": True,
    "use_issues_button": True,
    "use_edit_page_button": True,
}


def copy_markdown_files(app, exception):
    """Copy source markdown files"""
    if exception is None:  # Only run if build succeeded
        # Source dir is where your .md files are
        src_dir = app.srcdir  # This points to your `source/` folder
        dst_dir = app.outdir  # This is typically `build/html`

        for root, _, files in os.walk(src_dir):
            for file in files:
                if file.endswith(".md"):
                    src_path = os.path.join(root, file)
                    # Compute path relative to the source dir
                    rel_path = os.path.relpath(src_path, src_dir)
                    # Destination path inside the build output
                    dst_path = os.path.join(dst_dir, rel_path)

                    # Make sure the target directory exists
                    os.makedirs(os.path.dirname(dst_path), exist_ok=True)
                    shutil.copy2(src_path, dst_path)


def create_robots_txt(app, exception):
    """Create robots.txt file to take advantage of sitemap crawl"""
    if exception is None:
        dst_dir = app.outdir  # Typically 'build/html/'
        robots_path = os.path.join(dst_dir, "robots.txt")
        content = f"""User-agent: *

Sitemap: {html_baseurl}/sitemap.xml
"""
        with open(robots_path, "w") as f:
            f.write(content)


def setup(app):
    """Plugin to post build and copy markdowns as well"""
    app.connect("build-finished", copy_markdown_files)
    app.connect("build-finished", create_robots_txt)
