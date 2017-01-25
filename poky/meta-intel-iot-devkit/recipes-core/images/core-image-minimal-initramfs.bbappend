FILESEXTRAPATHS_prepend := "${THISDIR}/files:"

PACKAGE_INSTALL = "initramfs-live-boot busybox base-passwd udev"

PACKAGE_INSTALL_machine_quark += "kernel-module-usb-storage"
PACKAGE_INSTALL_machine_quark += "kernel-module-ehci-hcd kernel-module-ehci-pci kernel-module-ohci-hcd"
PACKAGE_INSTALL_machine_quark += "kernel-module-stmmac"
