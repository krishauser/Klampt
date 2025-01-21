# Minimal makefile for Sphinx documentation
#

# You can set these variables from the command line, and also
# from the environment for the first two.
SPHINXOPTS    ?= 
SPHINXBUILD   ?= sphinx-build
SOURCEDIR     = source
BUILDDIR      = _build

.PHONY: help html 

# Put it first so that "make" without argument is like "make help".
help: $(VENVDIR)
	@. $(VENV); $(SPHINXBUILD) -M help "$(SOURCEDIR)" "$(BUILDDIR)" $(SPHINXOPTS) $(O)

# Doesn't depend on $(BUILDDIR) to rebuild properly at every run.
html: 
	. $(VENV); $(SPHINXBUILD) -b dirhtml "$(SOURCEDIR)" "$(BUILDDIR)" -w .sphinx/warnings.txt $(SPHINXOPTS)

