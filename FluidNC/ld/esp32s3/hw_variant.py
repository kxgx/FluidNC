Import("env")

# User-facing releases have two functional choices (wifi / nowifi).  The
# hardware suffix is embedded in $I so Studio can choose the matching image
# automatically instead of asking the user for flash/PSRAM details.
VARIANT_BY_ENV = {
    "wifi_s3_oled_usb": "8mb-quad",
    "wifi_s3_octalPSRAM_oled_usb": "8mb-octal",
    "wifi_s3_octalPSRAM_oled_usb_16mb": "16mb-octal",
    "wifi_s3_oled_usb_4mb": "4mb-quad",
    "wifi_s3_octalPSRAM_oled_usb_4mb": "4mb-octal",
    "wifi_s3_oled_usb_4mb_nopsram": "4mb-nopsram",
    "noradio_s3_oled_usb": "8mb-quad",
    "noradio_s3_octalPSRAM_oled_usb": "8mb-octal",
    "noradio_s3_octalPSRAM_oled_usb_16mb": "16mb-octal",
    "noradio_s3_oled_usb_4mb": "4mb-quad",
    "noradio_s3_octalPSRAM_oled_usb_4mb": "4mb-octal",
    "noradio_s3_oled_usb_4mb_nopsram": "4mb-nopsram",
}

variant = VARIANT_BY_ENV.get(env.subst("$PIOENV"))
if variant:
    env.Append(CPPDEFINES=[("FLUIDCAN_HW", env.StringifyMacro(variant))])
