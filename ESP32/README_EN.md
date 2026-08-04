# FTDX10 Adapter Board (ESP32-S3) User Guide

The FTDX10 adapter board captures spectrum data from the FTDX10 transceiver (as an SPI slave) and forwards it over **USB CDC** and **WiFi** to host software (spectrum display, remote access, etc.).

---

## 1. Hardware Connection

| Transceiver (FTDX10) | ESP32-S3 Pin | Description |
|---|---|---|
| Pin5 (SCK) | GPIO10 | SPI clock (Mode 1, 2 MHz) |
| Pin4 (MOSI) | GPIO11 | SPI data (radio → adapter board) |
| Pin6 (CS) | GPIO13 | SPI chip select (active low, pull-up + glitch filtering) |
| Pin12 (radio TX) | GPIO38 | UART heartbeat RX (115200 8N1) |
| Pin11 (radio RX) | GPIO39 | UART heartbeat reply (reduced drive strength to prevent crosstalk) |
| — | GPIO0 (BOOT button) | Hold while powering on → WiFi provisioning mode |
| — | GPIO19/20 | USB-OTG data port (TinyUSB CDC) |
| — | Onboard USB-UART bridge | Log port (flashing & diagnostics) |

> **MISO is not required** (the adapter board only receives).
> After wiring, it is recommended to confirm normal SPI frames via `WIFI_STATUS` or the boot log (see Section 6).

---

## 2. Firmware Flashing (Arduino IDE)

1. **Board**: `ESP32S3 Dev Module` (or your corresponding S3 dev board model)
2. **Key board-level settings**:
   - **USB Mode = USB-OTG (Hardware CDC and JTAG)** — OTG is required, otherwise the USB data port will not work (the boot log will print `USB.begin() → FAILED`)
   - **USB CDC On Boot = Disabled** (the firmware initializes TinyUSB manually; both configurations compile, this one is the most stable)
   - Flash Size: 8 MB or 16 MB, according to your board
3. **Port**: select the COM port of the onboard USB-UART bridge (i.e. the log port)
4. **Baud rate**: 115200

After flashing, open the Serial Monitor (115200). You should see the boot banner:

```
╔═══════════════════════════════════╗
║  FTDX10 转接板  —  ESP32-S3      ║
╚═══════════════════════════════════╝
UART: GPIO38(RX)←Pin12 | GPIO39(TX)→Pin11 | 115200 8N1
SPI:  GPIO10(SCK)←Pin5 | GPIO11(MOSI)←Pin4 | GPIO13(CS)←Pin6
WiFi: TCP端口:51234 | UDP端口:51235 | 配网:USB命令/长按BOOT键
[USB] USB.begin() → OK (TinyUSB 已启动)
...
[WiFi] 就绪 | IP: 192.168.x.x | RSSI: -xx dBm
[UDP] 通道就绪 :51235 (等待 HELLO 注册)
```

---

## 3. WiFi Configuration

WiFi is **2.4 GHz only**.

### Provisioning via AP (Access Point)

Hold the **BOOT button** while powering on / resetting → the hotspot `FTDX10_AP` appears → connect to it with your phone → the captive portal opens automatically → enter your router SSID/password.

### Common Commands (sent via the log port / data port)

| Command | Function |
|---|---|
| `WIFI_SET <SSID> <PWD>` | Set WiFi and reboot |
| `WIFI_RESET` | Clear WiFi config and reboot |
| `WIFI_STATUS` | Query WiFi / link / UDP / TCP status |
| `HELP` | Show help |

---

## 4. Data Channels

| Channel | Port | Features | Use case |
|---|---|---|---|
| **USB CDC** | — (USB data COM port) | Full frame stream, stable and reliable | Wired direct connection |
| **WiFi UDP** | 51235 | **Full frame rate 25 fps**, no send-buffer limits, best real-time performance | Wireless main channel (recommended) |
| **WiFi TCP** | 51234 | Reliable and ordered, but limited by the firmware lwip send buffer (about 5–18 fps) | Wireless compatibility / backup |

**Frame format** (identical on all channels): `66 CC FF + length (2 bytes, big-endian) + XOR checksum (1 byte) + payload (4096 bytes)`

**UDP fragment protocol**: each frame is split into 3 datagrams (1366/1366/1364 bytes), each datagram = `66 CC FF + frame sequence (1 B) + fragment index (1 B) + payload`. After a client registers by sending `HELLO` to port 51235, the firmware unicasts to that source IP/port (random ports are supported if 51235 is occupied).

> For the detailed protocol, see `docs/protocol-zh.md`.

---

## 5. Usage

### GUI-less command-line relay (`PC_Software/remote.py`)

Suitable for scripted invocation and for feeding data into your own programs:

```bash
# UDP source → pipe to another program
python remote.py udp 192.168.1.100 | spectrum_program

# USB serial source → virtual serial port (for serial-based software such as wfview)
python remote.py usb COM3 --out com:CNCA0

# WiFi TCP source → local TCP server (other programs connect to 127.0.0.1:5200)
python remote.py tcp 192.168.1.100 --out tcp:5200

# Save data to a file
python remote.py udp 192.168.1.100 --out file:data.bin
```

Running without arguments enters an interactive mode that asks for the data source and the output sink. Outputs: `stdout | com:COMx | tcp:PORT | udp:PORT | file:PATH`. All outputs are a uniform full-frame stream (66 CC FF format).

---

## 6. Logs & Status

### Periodic status (one line every 5 s)
```
[WiFi] 5s状态: 客户1 发送+90 丢帧+36 UDP+125 RSSI-31 | 链路已连接 192.168.1.100
```
| Field | Meaning | Expected value |
|---|---|---|
| 客户 (clients) | Number of TCP clients | 0–4 |
| 发送+/丢帧+ (sent/dropped) | TCP frames sent/refused within 5 s | More drops = TCP buffer limitation |
| UDP+ | UDP frames sent within 5 s | **≈125 (full rate 25 fps)** |
| RSSI | Signal strength | Better than −60 dBm is good |

### Connection events
```
[UDP] 客户端192.168.1.50:51235 注册     ← UDP channel established ✓
[WiFi] 客户端192.168.1.50 已连接 → 槽0  ← TCP client joined
[WiFi] 槽0 缓冲满丢帧(降帧率)            ← TCP buffer full, rate reduced (normal; does not affect UDP)
[WiFi] 槽0 sndbuf不可查询 errno=109      ← Normal (precompiled library limit; UDP unaffected)
```

### Status query (send `WIFI_STATUS`)
```
[CMD] WiFi: 已连接 | IP: 192.168.1.100 | RSSI: -30 dBm | 链路: 正常 | 自愈: 0 | 队列丢: 0 | TCP发: 1523 | TCP客户: 0 | UDP发: 12500 | UDP丢片: 3 | UDP客户: 已注册
```

---

## 7. Troubleshooting

| Symptom | Check |
|---|---|
| No USB data COM port | ① Confirm USB Mode = USB-OTG; ② boot log shows `USB.begin()` OK; ③ refresh Device Manager / re-plug |
| Two COM ports, which is which | Data port = TinyUSB CDC (firmware USB data); log port = onboard USB-UART bridge (115200 boot messages) |
| No UDP data received by host software | ① Log shows `[UDP] 客户端... 注册`; ② IP is correct; ③ firewall allows UDP 51235 |
| UDP glitch / dropped frames | Weak signal or air interference — check RSSI (should be > −60 dBm); rule out 2.4 GHz interference; switch to TCP temporarily if needed |
| Spectrum checksum failures | SPI data issue: wiring correct, radio heartbeat replied (`REPLY_HEARTBEAT=1`), SPI mode = 1 |
| Radio stops sending data | Heartbeat reply timeout (must reply within 10 ms) — check UART wiring and `REPLY_HEARTBEAT=1` |
| WiFi drops repeatedly | Distance/interference; watch `掉线, 等待自动重连` frequency and RSSI in the log; firmware has auto-recovery (server rebuild) |

---

## 8. Firmware Configuration Quick Reference

| Setting | Value | Description |
|---|---|---|
| TCP port | 51234 | WiFi TCP server |
| UDP port | 51235 | WiFi UDP channel (HELLO registration) |
| Frame size | 4096 bytes | Single SPI frame |
| SPI | Mode 1 / 2 MHz / GPIO10,11,13 | Slave receive |
| UART heartbeat | 115200 / GPIO38,39 / `alive\0` | Must reply within 10 ms |
| `RUNTIME_LOG` | 0 | Runtime logging switch (1 = diagnostics, 0 = quiet) |
| `REPLY_HEARTBEAT` | 1 | Heartbeat reply (must be 1) |
| `HEARTBEAT_LED` | 0 | Heartbeat LED (GPIO48) switch |
| WiFi provisioning | Hold BOOT at power-on | 2.4 GHz only |
