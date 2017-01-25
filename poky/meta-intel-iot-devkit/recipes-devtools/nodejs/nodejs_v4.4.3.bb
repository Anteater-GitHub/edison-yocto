DESCRIPTION = "Node.js is a platform built on Chrome's JavaScript runtime for easily building fast, scalable network applications."
HOMEPAGE = "http://nodejs.org"
LICENSE = "MIT"
SECTION = "devel"
SUMMARY = "Node.js is a platform built on Chrome's JavaScript runtime for easily building fast, scalable network applications."
PR = "r1.0"
#DEPENDS = "openssl"
#DEPENDS_class-native += " openssl-native"

RCONFLICTS_${PN} = "node"

FILESEXTRAPATHS_prepend := "${THISDIR}/files:"
SRC_URI = "https://nodejs.org/dist/v4.4.3/node-${PV}.tar.gz \
           file://fix_ar.patch"

LIC_FILES_CHKSUM = "file://LICENSE;md5=96aa1ac122c41f8c08a0683d4b2126b5"

SRC_URI[md5sum] = "0f7feafa6a98af58fcd934c043ced0d5"
SRC_URI[sha256sum] = "8e67b95721aab7bd721179da2fe5dd97f9acc1306c15c9712ee103bcd6381638"

S = "${WORKDIR}/node-${PV}"

# v8 errors out if you have set CCACHE
CCACHE = ""

def map_dest_cpu(target_arch, d):
    import re
    if   re.match('i.86$', target_arch): return 'ia32'
    elif re.match('x86_64$', target_arch): return 'x64'
    return target_arch

def v8_target_arch(target_arch, d):
    import re
    if   re.match('i.86$', target_arch): return 'x87'
    elif re.match('x86_64$', target_arch): return 'x64'
    return target_arch

GYP_DEFINES_append_mipsel = " mips_arch_variant='r1' "

do_configure () {
  export LD="${CXX}"
  alias g++="${CXX}"
  GYP_DEFINES="${GYP_DEFINES}" export GYP_DEFINES
  ./configure --prefix=${prefix} --without-snapshot --dest-cpu=${@map_dest_cpu(d.getVar('TARGET_ARCH', True), d)} --dest-os=linux ${ARCHFLAGS}
  unalias g++
  cd ${S}; sed -i '/want_separate_host_toolset/ i \                \ \x27v8_target_arch\x27:\x27${@v8_target_arch(d.getVar('TARGET_ARCH', True), d)}\x27,' config.gypi
}

do_compile () {
  export LD="${CXX}"
  alias g++="${CXX}"
  make BUILDTYPE=Release
  unalias g++
}

do_install () {
  oe_runmake DESTDIR=${D} install
}

PACKAGES =+ "${PN}-npm"
FILES_${PN}-npm = "/usr/lib/node_modules ${bindir}/npm"
RDEPENDS_${PN}-npm = "bash python-compiler python-shell python-datetime python-subprocess python-multiprocessing python-crypt python-textutils python-netclient python-misc"

PACKAGES =+ "${PN}-dtrace"
FILES_${PN}-dtrace = "${libdir}/dtrace"

PACKAGES =+ "${PN}-systemtap"
FILES_${PN}-systemtap = "${datadir}/systemtap"

BBCLASSEXTEND = "native"
