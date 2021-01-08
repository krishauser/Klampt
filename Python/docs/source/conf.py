# -*- coding: utf-8 -*-
#
# Configuration file for the Sphinx documentation builder.
#
# This file does only contain a selection of the most common options. For a
# full list see the documentation:
# http://www.sphinx-doc.org/en/master/config

# -- Path setup --------------------------------------------------------------

# If extensions (or modules to document with autodoc) are in another directory,
# add these directories to sys.path here. If the directory is relative to the
# documentation root, use os.path.abspath to make it absolute, like shown here.
#
# import os
# import sys
import klampt
# sys.path.insert(0, os.path.abspath('.'))


# -- Project information -----------------------------------------------------

project = 'Klampt Python API'
copyright = '2020, Intelligent Motion Lab'
author = 'Kris Hauser'

# The short X.Y version
version = ''
# The full version, including alpha/beta/rc tags
release = '0.8.5'


# -- General configuration ---------------------------------------------------

# If your documentation needs a minimal Sphinx version, state it here.
#
# needs_sphinx = '1.0'

nitpicky = False
ignore_missing_types = ['bool','int','float','str','list','dict','tuple','function','None'
    'list of ints','list of strs','list of floats','list of 3 floats','list of 4 floats','list of 9 floats',
    'optional',
    'pair','pair of floats']
nitpick_ignore = [('py:class',t) for t in ignore_missing_types]

# Add any Sphinx extension module names here, as strings. They can be
# extensions coming with Sphinx (named 'sphinx.ext.*') or your custom
# ones.
extensions = [
    'sphinx.ext.autodoc',
    'sphinx.ext.viewcode',
    'sphinx.ext.mathjax',
    'sphinx_automodapi.automodapi',
    'sphinxcontrib.napoleon',
    'autodocsumm'
]

# Add any paths that contain templates here, relative to this directory.
templates_path = ['_templates']

autosummary_generate = True

#autodoc_default_options = {
#    'autosummary': True,
#}

# The suffix(es) of source filenames.
# You can specify multiple suffix as a list of string:
#
# source_suffix = ['.rst', '.md']
source_suffix = '.rst'

# The master toctree document.
master_doc = 'index'

# The language for content autogenerated by Sphinx. Refer to documentation
# for a list of supported languages.
#
# This is also used if you do content translation via gettext catalogs.
# Usually you set "language" from the command line for these cases.
language = None

# List of patterns, relative to source directory, that match files and
# directories to ignore when looking for source files.
# This pattern also affects html_static_path and html_extra_path.
exclude_patterns = []

# The name of the Pygments (syntax highlighting) style to use.
pygments_style = None


# -- Options for HTML output -------------------------------------------------

# The theme to use for HTML and HTML Help pages.  See the documentation for
# a list of builtin themes.
#
#html_theme = 'alabaster'
#KH: modified
html_theme = "sphinx_rtd_theme"

# Theme options are theme-specific and customize the look and feel of a theme
# further.  For a list of options available for each theme, see the
# documentation.
#
# html_theme_options = {}

# Add any paths that contain custom static files (such as style sheets) here,
# relative to this directory. They are copied after the builtin static files,
# so a file named "default.css" will overwrite the builtin "default.css".
html_static_path = ['_static']


#KH: modification for stupid table bug in RTD theme
html_context = {
    'css_files': [
        '_static/theme_overrides.css',  # override wide tables in RTD theme
        ],
     }


# Custom sidebar templates, must be a dictionary that maps document names
# to template names.
#
# The default sidebars (for documents that don't match any pattern) are
# defined by theme itself.  Builtin themes are using these templates by
# default: ``['localtoc.html', 'relations.html', 'sourcelink.html',
# 'searchbox.html']``.
#
# html_sidebars = {}


# -- Options for HTMLHelp output ---------------------------------------------

# Output file base name for HTML help builder.
htmlhelp_basename = 'KlamptPythonAPIdoc'


# -- Options for LaTeX output ------------------------------------------------

latex_elements = {
    # The paper size ('letterpaper' or 'a4paper').
    #
    # 'papersize': 'letterpaper',

    # The font size ('10pt', '11pt' or '12pt').
    #
    # 'pointsize': '10pt',

    # Additional stuff for the LaTeX preamble.
    #
    # 'preamble': '',

    # Latex figure (float) alignment
    #
    # 'figure_align': 'htbp',
}

# Grouping the document tree into LaTeX files. List of tuples
# (source start file, target name, title,
#  author, documentclass [howto, manual, or own class]).
latex_documents = [
    (master_doc, 'KlamptPythonAPI.tex', 'Klampt Python API Documentation',
     'Kris Hauser', 'manual'),
]


# -- Options for manual page output ------------------------------------------

# One entry per manual page. List of tuples
# (source start file, name, description, authors, manual section).
man_pages = [
    (master_doc, 'klamptpythonapi', 'Klampt Python API Documentation',
     [author], 1)
]


# -- Options for Texinfo output ----------------------------------------------

# Grouping the document tree into Texinfo files. List of tuples
# (source start file, target name, title, author,
#  dir menu entry, description, category)
texinfo_documents = [
    (master_doc, 'KlamptPythonAPI', 'Klampt Python API Documentation',
     author, 'KlamptPythonAPI', 'One line description of project.',
     'Miscellaneous'),
]


# -- Options for Epub output -------------------------------------------------

# Bibliographic Dublin Core info.
epub_title = project

# The unique identifier of the text. This can be a ISBN number
# or the project homepage.
#
# epub_identifier = ''

# A unique identification for the text.
#
# epub_uid = ''

# A list of files that should not be packed into the epub file.
epub_exclude_files = ['search.html']


# -- Extension configuration -------------------------------------------------

autoclass_content = 'both'






# KH: Added 1/8/2021
# ClassDocumenter.add_directive_header uses ClassDocumenter.add_line to
#   write the class documentation.
# We'll monkeypatch the add_line method and intercept lines that begin
#   with "Bases:".
# In order to minimize the risk of accidentally intercepting a wrong line,
#   we'll apply this patch inside of the add_directive_header method.

from sphinx.ext.autodoc import ClassDocumenter, _

add_line = ClassDocumenter.add_line
line_to_delete = _(u'Bases: %s') % u':class:`object`'

def add_line_no_object_base(self, text, *args, **kwargs):
    if text.strip() == line_to_delete:
        return
    add_line(self, text, *args, **kwargs)

add_directive_header = ClassDocumenter.add_directive_header

def add_directive_header_no_object_base(self, *args, **kwargs):
    self.add_line = add_line_no_object_base.__get__(self)
    result = add_directive_header(self, *args, **kwargs)
    del self.add_line
    return result

ClassDocumenter.add_directive_header = add_directive_header_no_object_base


