wch experiments

## WCH-Link

![](assets/WCH-Link.png)

https://www.wch.cn/products/WCH-Link.html

```log
wmesg -w

[ 1223.228032] usb 1-9.2: new full-speed USB device number 14 using xhci_hcd
[ 1223.333465] usb 1-9.2: New USB device found, idVendor=1a86, idProduct=8010, bcdDevice= 2.07
[ 1223.333497] usb 1-9.2: New USB device strings: Mfr=1, Product=2, SerialNumber=3
[ 1223.333509] usb 1-9.2: Product: WCH-Link
[ 1223.333517] usb 1-9.2: Manufacturer: wch.cn
[ 1223.333524] usb 1-9.2: SerialNumber: 0001A0000000
[ 1223.355473] cdc_acm 1-9.2:1.1: ttyACM0: USB ACM device
```

# udev

```
sudo cp 49-wch-link.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules && sudo udevadm trigger
```

## EVB

CH32Vx03C-R0-1v0

![](assets/CH32Vx03C-R0-1v0.png)

MCU: CH32V203C8T6

https://github.com/openwch/ch32v20x

| WCH-Link  | EVB   | MCU   |
|-----------|-------|-------|
| 3V3       | P2.4  | 3V3   |
| GND       | P2.6  | GND   |
| SWDIO     | P2.8  | PA13  |
| SWCLK     | P2.10 | PA14  |
| RST       | P2.12 | NRST  |

https://github.com/ch32-rs/wlink


```
cargo install --git https://github.com/ch32-rs/wlink
```

```
~/.cargo/bin/wlink status
```

* https://github.com/openwch/ch32v20x
* https://github.com/cnlohr/ch32v003fun

[ch32v20x.h](
https://github.com/openwch/ch32v20x/blob/main/EVT/EXAM/SRC/Peripheral/inc/ch32v20x.h)

