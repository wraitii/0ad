#!/bin/sh
set -ev

# Apply patches if needed
# This script gets called from build.sh.

# The rust code is only linked if the JS Shell is enabled,
# which fails now that rust is required in all cases.
# https://bugzilla.mozilla.org/show_bug.cgi?id=1588340
patch -p1 < ../FixRustLinkage.diff

# dllservices is not needed in JS Standalone, but is linked,
# and the files aren't bundled so this fails.
# https://bugzilla.mozilla.org/show_bug.cgi?id=1749306
patch -p1 < ../FixWindowsDLL.diff

# On windows, we need to differente debug/release library names.
patch -p1 < ../FixWindowsLibNames.diff

# There is an issue on 32-bit linux builds sometimes.
# NB: the patch here is Comment 21 modified by Comment 25
# but that seems to imperfectly fix the issue with GCC.
# https://bugzilla.mozilla.org/show_bug.cgi?id=1729459
patch -p1 < ../FixFpNormIssue.diff

