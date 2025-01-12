
# Work around for SUBDIR=../docs (or other non-subdir relative path)
top_builddir=$(abs_top_builddir)
top_srcdir=$(abs_top_srcdir)

MODULE_FILES=ar0823.ko
MODULE_NAME=d3-ar0823
DPKG_DIR=d3-ar0823
MODULE_VERSION=$(shell cat ./version.txt)

include $(D3_COMMON_PATH)/rules/module.defs
