# For the full list of built-in configuration values, see the documentation:
# https://www.sphinx-doc.org/en/master/usage/configuration.html

# -- Project information -----------------------------------------------------
# https://www.sphinx-doc.org/en/master/usage/configuration.html#project-information

project = "dyn2b"
copyright = "2023, Sven Schneider"
author = "Sven Schneider"

# -- General configuration ---------------------------------------------------
# https://www.sphinx-doc.org/en/master/usage/configuration.html#general-configuration

extensions = ["sphinx.ext.mathjax", "breathe"]

templates_path = ["_templates"]
exclude_patterns = []

numfig = True
math_numfig = True
numfig_format = {
    "figure": "Figure %s",
    "table": "Table %s",
    "code-block": "Listing %s",
    "section": "Section %s"
}

# -- Options for HTML output -------------------------------------------------
# https://www.sphinx-doc.org/en/master/usage/configuration.html#options-for-html-output

html_theme = "sphinx_rtd_theme"
html_static_path = ["_static"]
html_css_files = ["css/custom.css"]

# -- MathJax -----------------------------------------------------------------
# https://www.sphinx-doc.org/en/master/usage/extensions/math.html

mathjax_path = "https://cdn.jsdelivr.net/npm/mathjax@3/es5/tex-svg.js"

# -- Breathe -----------------------------------------------------------------
# https://breathe.readthedocs.io/en/latest/quickstart.html

breathe_domain_by_extension = {
    "c": "c",
    "h": "c"
}
