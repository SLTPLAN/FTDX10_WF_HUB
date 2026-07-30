# FTDX10 Adapter — Frame Protocol

中文版: https://github.com/SLTPLAN/FTDX10_WF_HUB/blob/main/README.md

This document defines the **data frame format** only. The transport layer can be USB-CDC, SPI, TCP/IP, etc., depending on the adapter's functionality and performance. 

Currently being implemented on STM32F103C8T6 and ESP32 via USB-CDC for use with a custom wfview build.

Download the wfview support FTDX10:https://tylk.cc/file_download/wfview-release-with-DX10-hub.zip

---

## Frame Format

```
┌───── 3 Bytes ─────┬──── 2 Bytes ────┬─── 1 Byte ──┬────── N Bytes ──────┐
│      Header        │   Payload Len   │  XOR Check  │       Payload       │
│  0x66 0xCC 0xFF    │  (Big-Endian)   │  Checksum   │      Payload       │
└────────────────────┴────────────────┴─────────────┴─────────────────────┘
```

| Field | Length | Description |
|:---|:---:|:---|
| Header | 3 B | Fixed `0x66 0xCC 0xFF`, frame sync marker |
| Length | 2 B | Big-endian, number of payload bytes |
| Checksum | 1 B | XOR checksum of payload |

---

## XOR Checksum

Computed over **all payload bytes**:

```
checksum = byte[0] ^ byte[1] ^ byte[2] ^ ... ^ byte[N-1]
```

The receiver re-computes XOR over the extracted payload and compares it with the checksum byte in the frame. A match means the frame is valid; a mismatch means the frame is corrupted or a false header match — the frame is discarded and parsing continues.

### Example

```
Payload (hex)   : 01 02 03 04
XOR checksum    : 01 ^ 02 ^ 03 ^ 04 = 04
Complete frame  : 66 CC FF 00 04 04 01 02 03 04
                    ^hdr^  ^len^ ^cs^ ^payload^
```

---

## Max Payload Length

| Parameter | Value | Description |
|:---|:---:|:---|
| `MAX_PAYLOAD_LEN` | **4600** | Maximum reasonable payload length |

The byte sequence `66 CC FF` may occasionally appear in raw SPI data, causing a false header match. For devices using USB-CDC, limiting the maximum payload length to 4600 (slightly above the actual max of 4096 ± 100) ensures:

- If the decoded length ≤ 4600 → parser waits, XOR check fails, **at most 1 frame lost**
- If the decoded length > 4600 → parser skips 1 byte immediately, **no frame loss**

---

## Transmission Strategy

The firmware splits 4KB frames into 64-byte chunks to avoid blocking the main loop during USB writes.

---

## Parsing Flow

```
Receive bytes → append to buffer
  ↓
Search for 66 CC FF in buffer
  ↓ (found)
Extract length field L [optional]
  ↓ L > 4600 ──→ skip 1 byte, re-search
  ↓ L ≤ 4600
Wait for L + 6 bytes to arrive
  ↓
Extract payload, compute XOR checksum
  ↓ checksum OK ──→ emit frame, trim buffer, continue
  ↓ checksum FAIL ──→ discard, trim buffer, continue
```

---

## Implementations

STM32F103C8T6
