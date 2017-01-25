SUMMARY = "Library that creates Bluetooth GATT bindings using BlueZ over DBus"
SECTION = "libs"
AUTHOR = "Petre Efitme, Andrei Vasiliu"

LICENSE = "MIT"
LIC_FILES_CHKSUM = "file://COPYING;md5=7edf6bde069df9c4192ac0f0a7572d72"

S = "${WORKDIR}/git"

inherit distutils-base pkgconfig cmake

PARALLEL_MAKE = "-j 1"

PACKAGECONFIG ??= "java"
PACKAGECONFIG[java] = "-DBUILDJAVA=ON, -DBUILDJAVA=OFF, openjdk-8, java-runtime"

export JAVA_HOME="${STAGING_LIBDIR}/jvm/java-8-openjdk"

DEPENDS_${PN} = "cmake (>= 3.1) glib-2.0 glib-2.0-native"
RDEPENDS_${PN} = "bluez5 (>= 5.37)"
