# Since the meta-intel-edison kernel recipe bypasses the YP kernel
# configuration, it's not possible to change the kernel config correctly.
# For now, s/r necessary kernel config lines in the defconfig provided by the
# meta-intel-edison layer
do_configure_append() {
  # Provide FTDI usb serial kernel support
  sed -i'' 's/.*CONFIG_USB_SERIAL is not set.*/CONFIG_USB_SERIAL=y/' ${B}/.config
  sed -i'' 's/.*CONFIG_USB_SERIAL_FTDI_SIO.*/CONFIG_USB_SERIAL_FTDI_SIO=m/' ${B}/.config

  # Provide realtek usbnet support
  sed -i'' 's/.*CONFIG_USB_USBNET.*/CONFIG_USB_USBNET=y/' ${B}/.config
  sed -i'' 's/.*CONFIG_USB_RTL8150.*/CONFIG_USB_RTL8150=m/' ${B}/.config
  sed -i'' 's/.*CONFIG_USB_RTL8152.*/CONFIG_USB_RTL8152=m/' ${B}/.config

  # Provide USB camera support for OV534
  sed -i'' 's/.*CONFIG_USB_GSPCA_OV534 is not set.*/CONFIG_USB_GSPCA_OV534=m/' ${B}/.config
  sed -i'' 's/.*CONFIG_USB_GSPCA_OV534_9 is not set.*/CONFIG_USB_GSPCA_OV534_9=m/' ${B}/.config
}
