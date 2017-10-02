#
# Yocto recipe to build a kernel module out of the kernel tree
# hellokernel.bb  
# Yuncheng Song 
#

DESCRIPTION = "Hello kernel module out of the kernel tree"
SECTION = "examples"
LICENSE = "GPLv2"
LIC_FILES_CHKSUM = "file://COPYING;md5=b801565caf0fdbb2230909c852d84fdb"
PR = "r0"

inherit module

SRC_URI = "file://hellokernel.c file://Makefile file://COPYING "

S = "${WORKDIR}"