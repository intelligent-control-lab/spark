SPHINXBUILD ?= sphinx-build
SOURCEDIR = .
BUILDDIR = _build

.PHONY: help html livehtml clean linkcheck

help:
	@echo "html       build production HTML"
	@echo "livehtml   rebuild and refresh a local preview"
	@echo "linkcheck  check external and internal links"
	@echo "clean      remove generated files"

html:
	$(SPHINXBUILD) -W --keep-going -d "$(BUILDDIR)/doctrees" -b html "$(SOURCEDIR)" "$(BUILDDIR)/html"

livehtml:
	sphinx-autobuild --open-browser "$(SOURCEDIR)" "$(BUILDDIR)/html"

linkcheck:
	$(SPHINXBUILD) -W --keep-going -d "$(BUILDDIR)/doctrees" -b linkcheck "$(SOURCEDIR)" "$(BUILDDIR)/linkcheck"

clean:
	rm -rf "$(BUILDDIR)"
