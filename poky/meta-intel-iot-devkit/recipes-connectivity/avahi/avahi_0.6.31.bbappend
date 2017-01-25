inherit systemd

FILESEXTRAPATHS_prepend := "${THISDIR}/files:"

SRC_URI += "file://avahi-autoipd.service \
            file://avahi-autoipd-auto"

do_install_append () {
  cp ${WORKDIR}/avahi-autoipd-auto ${D}${sbindir}

  install -d ${WORKDIR}${systemd_unitdir}/system/
  install -m 0644 ${WORKDIR}/avahi-autoipd.service ${D}${systemd_unitdir}/system/
}

SYSTEMD_PACKAGES += "${PN}-autoipd"

# using ${PN} in the following creates a variable shadowing warning
FILES_avahi-autoipd_append += " ${systemd_unitdir}/system/avahi-autoipd.service"

SYSTEMD_SERVICE_${PN}-autoipd = "avahi-autoipd.service"
