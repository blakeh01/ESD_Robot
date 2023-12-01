import os
import sys

# Add the project's root directory to the system path
sys.path.insert(0, os.path.abspath('..'))

# -- Project information -----------------------------------------------------

project = 'ESD Apparatus'
author = ''

# -- General configuration ---------------------------------------------------

# Add any Sphinx extension module names here
extensions = [
    'sphinx.ext.autodoc',
    'sphinx.ext.napoleon',
]

# Napoleon settings for handling Google-style docstrings
napoleon_google_docstring = True
napoleon_numpy_docstring = False

# -- Options for HTML output -------------------------------------------------

# Theme options are theme-specific and customize the look and feel
html_theme = 'alabaster'
html_theme_options = {
    'logo': 'your_logo.png',
    'github_user': 'your_username',
    'github_repo': 'your_repo',
    'github_banner': True,
    'show_powered_by': False,
}

# -- Options for LaTeX output ------------------------------------------------

latex_elements = {
    # The paper size ('letterpaper' or 'a4paper').
    'papersize': 'letter',

    # The font size ('10pt', '11pt' or '12pt').
    'pointsize': '10',

    # Additional stuff for the LaTeX preamble.
    'preamble': '',
}

# -- Extension configuration -------------------------------------------------

# Add any paths that contain templates here, relative to this directory.
templates_path = ['_templates']

# The master toctree document.
master_doc = 'index'

# -- Options for autodoc extension -------------------------------------------

# Generate autodoc stubs with summaries
autodoc_default_options = {
    'members': None,
    'member-order': 'bysource',
    'show-inheritance': False,
}

# -- Options for Napoleon extension ------------------------------------------

# Napoleon settings for handling Google-style docstrings
napoleon_use_rtype = False
napoleon_use_param = False
napoleon_use_ivar = True

# -- Options for HTMLHelp output ---------------------------------------------

# Output file base name for HTML help builder.
htmlhelp_basename = 'ESD_RBT'

# -- Options for LaTeX output ------------------------------------------------

# Grouping the document tree into LaTeX files.
latex_documents = [
    (master_doc, 'ESDRBT.tex', 'ESD Apparatus Documentation',
     'Blake Havens', 'manual'),
]

# -- Options for manual page output ------------------------------------------

# One entry per manual page. List of tuples
man_pages = [
    (master_doc, 'ESD_RBT', 'ESD Apparatus Documentation',
     [author], 1)
]

# -- Options for Texinfo output ----------------------------------------------

# Grouping the document tree into Texinfo files.
texinfo_documents = [
    (master_doc, 'ESD_RBT', 'ESD Apparatus Documentation',
     author, 'ESD_RBT', 'Python controller code for ESD apparatus.',
     'Miscellaneous'),
]