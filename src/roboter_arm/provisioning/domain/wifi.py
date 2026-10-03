"""Wi-Fi credentials a NetworkManager profile on the Pi can hold."""


def check_wifi(ssid, psk):
    # NetworkManager keyfiles give ";" a special meaning in SSIDs.
    if not ssid or len(ssid.encode()) > 32 or not ssid.isprintable() or ';' in ssid:
        raise ValueError('Set WIFI_SSID in .env: 1-32 bytes of printable text without ";"')
    if not psk or not 8 <= len(psk) <= 63 or not (psk.isascii() and psk.isprintable()):
        raise ValueError('Set WIFI_PASS in .env: a WPA passphrase of 8-63 printable ASCII characters')
