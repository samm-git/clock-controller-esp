## v0.0.3
- Improve wifi reconnection reliability (event handler, auto-reconnect, disable modem sleep)
- Add boot timeouts for wifi and NTP so the device never hangs without network
- Only move the slave clock once a valid time is available
- Check DNS lookup result before sending NTP packet
- Replace per-second String allocations with snprintf buffers
- Fix misleading comment about clock being ahead/behind NTP
- do not call updateScreen if there is nothing to update
- Fix UI for the long WIFI SSID-s

## v0.0.2
- Create screensaver to prolong OLED life
- Use GPIO 15 in a touch-button mode

## v0.0.1
- First initial release
