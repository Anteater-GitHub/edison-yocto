FILESEXTRAPATHS_prepend := "${THISDIR}/files:"

SRC_URI += "file://iotkit-comm-js.mdns.patch"

PACKAGES =+ "${PN}-mdns"
FILES_${PN}-mdns += "${libdir}/node_modules/iotkit-comm/node_modules/mdns"

do_install_append () {
    echo "YYYXXX ${FILES_${PN}-mdns}"
    echo "YYYXXX ${libdir}"
}
