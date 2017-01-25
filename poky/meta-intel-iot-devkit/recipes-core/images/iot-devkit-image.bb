DESCRIPTION = "A fully functional image to be placed on an SD card"

IMAGE_INSTALL = "packagegroup-core-boot ${ROOTFS_PKGMANAGE_BOOTSTRAP} ${CORE_IMAGE_EXTRA_INSTALL}"

IMAGE_LINGUAS = " "

LICENSE = "GPLv2"

IMAGE_FSTYPES = "ext3 live"

inherit core-image

NOISO = "1"
IMAGE_ROOTFS_SIZE = "307200"

EXTRA_IMAGECMD_append_ext2 = " -N 2000"

IMAGE_FEATURES += "package-management"
IMAGE_INSTALL += "kernel-modules"
IMAGE_INSTALL += "ethtool"
IMAGE_INSTALL += "strace"
IMAGE_INSTALL += "ppp"
IMAGE_INSTALL += "linuxptp"
IMAGE_INSTALL += "libstdc++"
IMAGE_INSTALL += "sysstat"

IMAGE_INSTALL += "python python-modules python-numpy"
IMAGE_INSTALL += "alsa-lib alsa-utils"
IMAGE_INSTALL += "wireless-tools"
IMAGE_INSTALL += "wpa-supplicant"
IMAGE_INSTALL += "openssh"
IMAGE_INSTALL += "nodejs"

IMAGE_INSTALL += "openjdk-8-jdk"

IMAGE_INSTALL += "linux-firmware-iwlwifi-6000g2a-6"
IMAGE_INSTALL += "linux-firmware-iwlwifi-135-6"
IMAGE_INSTALL += "bluez5"
IMAGE_INSTALL += "avahi avahi-autoipd"
IMAGE_INSTALL += "fuse-utils"
IMAGE_INSTALL += "connman connman-client connman-tests"
IMAGE_INSTALL += "tzdata"
IMAGE_INSTALL += "ca-certificates"
IMAGE_INSTALL += "icu"
IMAGE_INSTALL += "opencv"
IMAGE_INSTALL += "swig"
IMAGE_INSTALL += "lighttpd"

IMAGE_INSTALL += "mraa upm"
IMAGE_INSTALL += "timedate-scripts"
IMAGE_INSTALL += "iotkit-agent"
IMAGE_INSTALL += "xdk-daemon"
IMAGE_INSTALL += "openjdk-8-jre"
IMAGE_INSTALL += "mdns"
IMAGE_INSTALL += "wyliodrin-server"
IMAGE_INSTALL += "python-pip"

IMAGE_INSTALL += "iotivity"
#IMAGE_INSTALL += "iotivity-dev"
#IMAGE_INSTALL += "iotivity-tests"
#IMAGE_INSTALL += "iotivity-plugins-staticdev"
#IMAGE_INSTALL += "iotivity-plugins-samples"
#IMAGE_INSTALL += "iotivity-resource"
#IMAGE_INSTALL += "iotivity-resource-dev"
#IMAGE_INSTALL += "iotivity-resource-thin-staticdev"
#IMAGE_INSTALL += "iotivity-resource-samples"
#IMAGE_INSTALL += "iotivity-service"
#IMAGE_INSTALL += "iotivity-service-dev"
#IMAGE_INSTALL += "iotivity-service-staticdev"
#IMAGE_INSTALL += "iotivity-service-samples"
#IMAGE_INSTALL += "iotivity-simple-client"

IMAGE_INSTALL += "packagegroup-core-eclipse-debug"

IMAGE_INSTALL += "tinyb"

# these are the only lib32-* libs we want on our image
IMAGE_INSTALL_append_quark += "lib32-uclibc lib32-uclibc-libm lib32-libstdc++ lib32-uclibc-libpthread"
# make sure no lib32-* libs get chosen by IMAGE_FEATURES
PACKAGE_EXCLUDE_COMPLEMENTARY = "lib32-.*"

ROOTFS_POSTPROCESS_COMMAND_append_quark += "symlink_ld_uclibc ; install_quark_repo ; install_wyliodrin ; "
ROOTFS_POSTPROCESS_COMMAND_append_intel-core2-32 += "install_mmax_repo ;"
ROOTFS_POSTPROCESS_COMMAND += "install_xdk ; symlink_node_modules ; "

symlink_ld_uclibc() {
  # This allows uclibc compiled binaries to find the uclibc loader note that
  # binaries will not run unless LD_LIBRARY_PATH is set correctly
  cd ${IMAGE_ROOTFS}/lib/; ln -s ../lib32/ld-uClibc.so.0
  # Remove g_serial.conf as we use another module
  rm -f ${IMAGE_ROOTFS}/etc/modules-load.d/g_serial.conf
  # make sure g_acm_ms loads last
  mv ${IMAGE_ROOTFS}/etc/modules-load.d/g_acm_ms.conf ${IMAGE_ROOTFS}/etc/modules-load.d/z_g_acm_ms.conf
}

install_mmax_repo() {
  echo "src mraa-upm http://iotdk.intel.com/repos/3.5/intelgalactic/opkg/i586/" > ${IMAGE_ROOTFS}/etc/opkg/mraa-upm.conf
  echo "src iotdk-all http://iotdk.intel.com/repos/3.5/iotdk/mmax/all" > ${IMAGE_ROOTFS}/etc/opkg/iotdk.conf
  echo "src iotdk-i586 http://iotdk.intel.com/repos/3.5/iotdk/mmax/core2-32" >> ${IMAGE_ROOTFS}/etc/opkg/iotdk.conf
  echo "src iotdk-quark http://iotdk.intel.com/repos/3.5/iotdk/mmax/core2-32-intel-common" >> ${IMAGE_ROOTFS}/etc/opkg/iotdk.conf
  echo "src iotdk-x86 http://iotdk.intel.com/repos/3.5/iotdk/mmax/intel_core2_32" >> ${IMAGE_ROOTFS}/etc/opkg/iotdk.conf
}

install_quark_repo() {
  echo "src mraa-upm http://iotdk.intel.com/repos/3.5/intelgalactic/opkg/i586/" > ${IMAGE_ROOTFS}/etc/opkg/mraa-upm.conf
  echo "src iotdk-all http://iotdk.intel.com/repos/3.5/iotdk/galileo/all" > ${IMAGE_ROOTFS}/etc/opkg/iotdk.conf
  echo "src iotdk-i586 http://iotdk.intel.com/repos/3.5/iotdk/galileo/i586" >> ${IMAGE_ROOTFS}/etc/opkg/iotdk.conf
  echo "src iotdk-quark http://iotdk.intel.com/repos/3.5/iotdk/galileo/quark" >> ${IMAGE_ROOTFS}/etc/opkg/iotdk.conf
  echo "src iotdk-x86 http://iotdk.intel.com/repos/3.5/iotdk/galileo/x86" >> ${IMAGE_ROOTFS}/etc/opkg/iotdk.conf
}

install_xdk() {
  # Create app_slot dir for XDK daemon
  install -d ${IMAGE_ROOTFS}/node_app_slot
}

symlink_node_modules() {
  # Create symlink from /usr/lib/node_modules/ to /usr/lib/node/ as different
  # people seem to want different paths
  cd ${IMAGE_ROOTFS}/usr/lib/; ln -s node_modules node
}

install_wyliodrin() {
  # Wyliodrin requires the boot partition to be mounted as /media/card
  install -d ${IMAGE_ROOTFS}/media/card
  # to maintain compatibility with arduino SD lib
  cd ${IMAGE_ROOTFS}/media; ln -s card mmcblk0p1
  # we add a line in fstab to automount the uSD card /boot partition
  echo "/dev/mmcblk0p1 /media/card auto defaults 0  0" >> ${IMAGE_ROOTFS}/etc/fstab
  echo -n "arduinogalileo" > ${IMAGE_ROOTFS}/etc/wyliodrin/boardtype
  mkdir -p ${IMAGE_ROOTFS}/wyliodrin/projects/mount
  mkdir -p ${IMAGE_ROOTFS}/wyliodrin/projects/build
}

EXTRA_IMAGEDEPENDS_append_quark = " grub-conf "

