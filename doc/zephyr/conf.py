#
# Copyright (c) 2025 Nordic Semiconductor
#
# SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
#

# Zephyr documentation build configuration file

import os
from pathlib import Path
import sys
from sphinx.config import eval_config_file


# Paths ------------------------------------------------------------------------

NRF_BASE = Path(__file__).absolute().parents[2]

sys.path.insert(0, str(NRF_BASE / "doc" / "_utils"))
import utils

ZEPHYR_BASE = utils.get_projdir("zephyr")

# pylint: disable=undefined-variable,used-before-assignment

# General ----------------------------------------------------------------------

# Import all Zephyr configuration, override as needed later
os.environ["ZEPHYR_BASE"] = str(ZEPHYR_BASE)
os.environ["OUTPUT_DIR"] = str(utils.get_builddir() / "html" / "zephyr")

conf = eval_config_file(str(ZEPHYR_BASE / "doc" / "conf.py"), tags)
locals().update(conf)

sys.path.insert(0, str(NRF_BASE / "doc" / "_extensions"))

extensions.remove("sphinx_sitemap")
extensions = ["sphinx.ext.intersphinx"] + extensions

# Options for HTML output ------------------------------------------------------

html_additional_pages = {}
html_theme = "sphinx_ncs_theme"
html_theme_path = []
html_favicon = None
html_static_path.append(str(NRF_BASE / "doc" / "_static"))
html_last_updated_fmt = "%b %d, %Y"
html_show_sourcelink = True
html_logo = None
html_title = "Zephyr Project documentation (nRF Connect SDK)"

html_context = {
    "show_license": True,
    "docs_title": docs_title,
    "is_release": is_release,
}

html_theme_options = {
    "docset": "zephyr",
    "docsets": utils.ALL_DOCSETS,
    "subtitle": "nRF Connect SDK",
}

# Options for intersphinx ------------------------------------------------------

intersphinx_mapping = dict()

kconfig_mapping = utils.get_intersphinx_mapping("kconfig")
if kconfig_mapping:
    intersphinx_mapping["kconfig"] = kconfig_mapping

# -- Options for doxyrunner plugin ---------------------------------------------

_doxyrunner_outdir = utils.get_builddir() / "html" / "zephyr" / "doxygen"

doxyrunner_doxygen = os.environ.get("DOXYGEN_EXECUTABLE", "doxygen")
doxyrunner_projects = {
    "zephyr": {
        "doxyfile": NRF_BASE / "doc" / "zephyr" / "zephyr.doxyfile.in",
        "outdir": _doxyrunner_outdir,
        "fmt": True,
        "fmt_vars": {
            "NRF_BASE": str(NRF_BASE),
            "DOCSET_SOURCE_BASE": str(ZEPHYR_BASE),
            "DOCSET_BUILD_DIR": str(_doxyrunner_outdir),
            "DOCSET_VERSION": version,
        }
    }
}

# -- Options for doxybridge plugin ---------------------------------------------

doxybridge_projects = {"zephyr": _doxyrunner_outdir}

# -- Options for zephyr.warnings_filter ----------------------------------------

warnings_filter_silent = True

# -- Options for zephyr.kconfig ------------------------------------------------

kconfig_generate_db = False

# -- Options for zephyr.gh_utils -----------------------------------------------

gh_link_version = "main" if version.endswith("99") else f"v{version}"
gh_link_base_url = f"https://github.com/nrfconnect/sdk-zephyr"

# pylint: enable=undefined-variable,used-before-assignment


def setup(app):
    app.add_css_file("css/zephyr.css")

    utils.add_google_analytics(app, html_theme_options)
    utils.add_announcement_banner(html_theme_options)
