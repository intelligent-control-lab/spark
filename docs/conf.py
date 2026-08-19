from __future__ import annotations

project = "SPARK"
author = "SPARK Team"
copyright = "2026, SPARK Team"
version = "2.0"
release = "2.0.0"

extensions = [
    "myst_parser",
    "sphinx.ext.autosectionlabel",
    "sphinx.ext.intersphinx",
    "sphinx.ext.napoleon",
    "sphinx_copybutton",
    "sphinx_design",
    "sphinxcontrib.mermaid",
]

source_suffix = {".md": "markdown"}
master_doc = "index"
exclude_patterns = [
    "_build",
    "Thumbs.db",
    ".DS_Store",
    "architecture/policy_and_runtime_architecture.md",
    "architecture/policy_migration.md",
    "reference/contributing_docs.md",
    "modules/agent_environment.md",
    "modules/safety.md",
    "modules/_generated/**",
    "tutorials/unitree_g1.md",
    "tutorials/other_robots.md",
]
nitpicky = True
autosectionlabel_prefix_document = True
# Remote inventories improve API links but should not make an otherwise local
# documentation build depend on third-party network availability.
suppress_warnings = ["intersphinx.external"]

myst_enable_extensions = [
    "colon_fence",
    "deflist",
    "fieldlist",
    "substitution",
    "tasklist",
]

html_theme = "furo"
html_title = "SPARK Documentation"
html_logo = "img/SPARK_logo.png"
html_favicon = "img/SPARK_logo.png"
html_static_path = ["_static"]
html_css_files = ["spark.css"]
html_js_files = ["navigation.js"]
html_theme_options = {
    "light_css_variables": {
        "color-brand-primary": "#d04a1d",
        "color-brand-content": "#b83b13",
    },
    "dark_css_variables": {
        "color-brand-primary": "#ff8b5b",
        "color-brand-content": "#ff9c76",
    },
    "source_repository": "https://github.com/intelligent-control-lab/spark/",
    "source_branch": "master",
    "source_directory": "docs/",
}

intersphinx_mapping = {
    "python": ("https://docs.python.org/3", None),
    "numpy": ("https://numpy.org/doc/stable/", None),
}

linkcheck_ignore = [r"http://localhost:\d+/.*"]
