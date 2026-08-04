#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
FTDX10 Adapter Board Remote Data Relay (GUI-less, scriptable)
=============================================================
Forward FTDX10 adapter board data streams (USB serial / WiFi UDP / WiFi TCP) to:
  stdout (pipe to other programs) | virtual serial port (com0com) |
  local TCP server | UDP | file

All channels output a unified full-frame stream (66 CC FF + len(2B) + XOR + payload);
downstream programs do not need to know the data source.

Usage:
  python remote_en.py <mode> <addr> [--out <sink>] [--baud N] [--port N]

  mode: usb | udp | tcp
  addr: COMx (usb) | adapter IP (udp/tcp)
  --out (default stdout):
    stdout             standard output (pipe/redirect, e.g. | spectrum_program)
    com:COMx           virtual serial port (com0com port, e.g. com:CNCA0)
    tcp:PORT           local TCP server (other programs connect to 127.0.0.1:PORT)
    udp:PORT           local UDP forward (other programs listen on 127.0.0.1:PORT)
    file:PATH          write to file (e.g. file:spectrum.bin)
  --baud: USB serial baud rate (default 115200; USB CDC virtual baud has no real limit)
  --port: TCP source port (default 51234, firmware TCP_PORT)

Examples:
  python remote_en.py udp 192.168.1.100 | spectrum_program
  python remote_en.py usb COM3 --out com:CNCA0
  python remote_en.py tcp 192.168.1.100 --out tcp:5200
  python remote_en.py udp 192.168.1.100 --out file:data.bin

Run without arguments → interactive selection of data source and output sink.
"""

import argparse
import socket
import sys
import threading
import time
from collections import deque

FRAME_MAGIC = b"\x66\xcc\xff"
UDP_PORT_DEFAULT = 51235   # firmware UDP channel port (matches PC software UDP_PORT_DEFAULT)
TCP_PORT_DEFAULT = 51234   # firmware TCP server port
BAUD_DEFAULT = 115200

try:
    import serial  # pyserial: only needed for usb source / com output
except ImportError:
    serial = None


# ============================================================
#  Sink — thread-safe, drop-oldest on full queue, never blocks the source
# ============================================================
class StdoutSink:
    """stdout (pipe): flush each packet, exit if the downstream pipe breaks"""

    def write(self, data: bytes):
        try:
            sys.stdout.buffer.write(data)
            sys.stdout.buffer.flush()
        except BrokenPipeError:
            raise SystemExit(0)

    def close(self):
        pass


class SerialSink:
    """Virtual serial port output (com0com): bounded queue + writer thread, drop-oldest on full"""

    def __init__(self, port: str, baud: int = BAUD_DEFAULT):
        if serial is None:
            sys.exit("[ERR] pyserial missing: pip install pyserial")
        try:
            self._ser = serial.Serial(port, baud, timeout=0)
        except Exception as e:
            sys.exit(f"[ERR] cannot open serial port {port}: {e}")
        self._q = deque(maxlen=128)
        self._stop = threading.Event()
        self._th = threading.Thread(target=self._run, daemon=True)
        self._th.start()
        print(f"[OUT] virtual serial port {port} @{baud}")

    def write(self, data: bytes):
        self._q.append(data)   # maxlen full → auto drop-oldest (non-blocking)

    def _run(self):
        while not self._stop.is_set():
            try:
                b = self._q.popleft()
            except IndexError:
                time.sleep(0.002)
                continue
            try:
                self._ser.write(b)
            except Exception as e:
                print(f"[ERR] serial write failed: {e}", file=sys.stderr)
                break

    def close(self):
        self._stop.set()
        try:
            self._ser.close()
        except Exception:
            pass


class TcpServerSink:
    """Local TCP server: multi-client broadcast, drop when no client connected"""

    def __init__(self, port: int):
        self._srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self._srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self._srv.bind(("127.0.0.1", port))
        self._srv.listen(4)
        self._clients = set()
        self._lock = threading.Lock()
        threading.Thread(target=self._accept, daemon=True).start()
        print(f"[OUT] TCP server 127.0.0.1:{port}")

    def _accept(self):
        while True:
            try:
                c, _ = self._srv.accept()
            except OSError:
                break
            try:
                c.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
                c.settimeout(5.0)
            except OSError:
                pass
            with self._lock:
                self._clients.add(c)

    def write(self, data: bytes):
        dead = []
        with self._lock:
            for c in list(self._clients):
                try:
                    c.sendall(data)
                except OSError:
                    dead.append(c)
            for c in dead:
                self._clients.discard(c)
                try:
                    c.close()
                except OSError:
                    pass

    def close(self):
        with self._lock:
            for c in list(self._clients):
                try:
                    c.close()
                except OSError:
                    pass
            self._clients.clear()
        try:
            self._srv.close()
        except OSError:
            pass


class UdpSink:
    """Local UDP forward"""

    def __init__(self, port: int):
        self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._port = port
        print(f"[OUT] UDP forward 127.0.0.1:{port}")

    def write(self, data: bytes):
        try:
            self._sock.sendto(data, ("127.0.0.1", self._port))
        except OSError:
            pass

    def close(self):
        try:
            self._sock.close()
        except OSError:
            pass


class FileSink:
    """File output (append mode)"""

    def __init__(self, path: str):
        self._f = open(path, "ab", buffering=0)
        print(f"[OUT] file {path}")

    def write(self, data: bytes):
        self._f.write(data)

    def close(self):
        try:
            self._f.close()
        except OSError:
            pass


def build_sink(spec: str):
    """Parse --out spec → sink object"""
    spec = (spec or "stdout").strip()
    if spec == "stdout":
        return StdoutSink()
    if spec.startswith("com:"):
        return SerialSink(spec[4:])
    if spec.startswith("tcp:"):
        return TcpServerSink(int(spec[4:]))
    if spec.startswith("udp:"):
        return UdpSink(int(spec[4:]))
    if spec.startswith("file:"):
        return FileSink(spec[5:])
    sys.exit(f"[ERR] unrecognized output: {spec} (stdout|com:COMx|tcp:PORT|udp:PORT|file:PATH)")


# ============================================================
#  Source — always outputs a unified full-frame stream (66 CC FF ...)
# ============================================================
def make_frame(payload: bytes) -> bytes:
    """Re-wrap a reassembled UDP payload as a full frame (same as USB/TCP channels)"""
    cs = 0
    for b in payload:
        cs ^= b
    return FRAME_MAGIC + len(payload).to_bytes(2, "big") + bytes([cs]) + payload


def usb_source(port: str, baud: int, sink, stop: threading.Event):
    """USB serial source: raw byte stream (firmware already wraps full frames)"""
    if serial is None:
        print("[ERR] pyserial missing: pip install pyserial", file=sys.stderr)
        stop.set()
        return
    try:
        ser = serial.Serial(port, baud, timeout=0.1)
    except Exception as e:
        print(f"[ERR] cannot open serial port {port}: {e}", file=sys.stderr)
        stop.set()
        return
    print(f"[SRC] USB serial {port} @{baud} (Ctrl+C to quit)")
    while not stop.is_set():
        try:
            n = ser.in_waiting
            if n > 0:
                data = ser.read(n)
                if data:
                    sink.write(data)
            else:
                time.sleep(0.001)
        except Exception as e:
            print(f"[ERR] serial read failed: {e}", file=sys.stderr)
            break
    ser.close()


def udp_source(host: str, sink, stop: threading.Event):
    """WiFi UDP source: HELLO register → fragment reassembly → full frames out"""
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    try:
        sock.bind(("", UDP_PORT_DEFAULT))
    except OSError:
        sock.bind(("", 0))   # port busy → random port (firmware replies to source port)
    sock.settimeout(1.0)
    try:
        sock.sendto(b"HELLO", (host, UDP_PORT_DEFAULT))
    except OSError as e:
        print(f"[ERR] HELLO send failed: {e}", file=sys.stderr)
        sock.close()
        stop.set()
        return
    print(f"[SRC] UDP registered {host}:{UDP_PORT_DEFAULT} (Ctrl+C to quit)")

    seq, parts = -1, [None] * 3
    ok = lost = 0
    last_data = time.time()
    while not stop.is_set():
        try:
            dg, _ = sock.recvfrom(2048)
        except socket.timeout:
            if time.time() - last_data > 2.0:
                print("[UDP] no data for 2s (radio off? adapter not running or wrong IP?)", file=sys.stderr)
                last_data = time.time()
            continue
        except OSError:
            break
        last_data = time.time()
        if len(dg) < 5 or dg[:3] != FRAME_MAGIC:
            continue
        s, sl = dg[3], dg[4]
        if sl >= 3:
            continue
        if s != seq:
            if seq >= 0 and any(p is not None for p in parts):
                lost += 1
            seq, parts = s, [None] * 3
        parts[sl] = dg[5:]
        if all(p is not None for p in parts):
            frame = b"".join(parts)
            parts = [None] * 3
            if len(frame) == 4096:
                ok += 1
                sink.write(make_frame(frame))
            else:
                print(f"[UDP] reassembly length mismatch {len(frame)}", file=sys.stderr)
    if ok or lost:
        print(f"[UDP] stats: {ok} frames complete, {lost} dropped (missing fragments)", file=sys.stderr)
    sock.close()


def tcp_source(host: str, port: int, sink, stop: threading.Event):
    """WiFi TCP source: auto-reconnect on drop, raw frame stream"""
    while not stop.is_set():
        try:
            s = socket.create_connection((host, port), timeout=5)
        except OSError as e:
            print(f"[TCP] connect failed: {e}, retry in 3s (Ctrl+C to quit)", file=sys.stderr)
            for _ in range(30):
                if stop.is_set():
                    return
                time.sleep(0.1)
            continue
        s.settimeout(1.0)
        try:
            s.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        except OSError:
            pass
        print(f"[SRC] TCP connected {host}:{port} (Ctrl+C to quit)")
        while not stop.is_set():
            try:
                data = s.recv(65536)
            except socket.timeout:
                continue
            except OSError:
                break
            if not data:
                print("[TCP] connection lost, reconnecting...", file=sys.stderr)
                break
            sink.write(data)
        s.close()


# ============================================================
#  Interactive selection (run without arguments)
# ============================================================
def _list_ports():
    if serial is None:
        return []
    try:
        import serial.tools.list_ports as lp
        return [p.device for p in lp.comports()]
    except Exception:
        return []


def interactive_source():
    print("=== FTDX10 Adapter Remote Data Relay ===")
    print("Data source:")
    print("  1. USB serial")
    print("  2. WiFi UDP (port 51235)")
    print("  3. WiFi TCP (port 51234)")
    ch = input("Select (1-3): ").strip()
    if ch == "1":
        ports = _list_ports()
        if ports:
            print("Serial ports detected:")
            for i, p in enumerate(ports, 1):
                print(f"    {i}. {p}")
            sel = input(f"Select (1-{len(ports)}) or type a port name [{', '.join(ports[:3])}...]: ").strip()
            if sel.isdigit() and 1 <= int(sel) <= len(ports):
                port = ports[int(sel) - 1]
            else:
                port = sel or ports[0]
        else:
            port = input("No ports detected, type a port name manually (e.g. COM3): ").strip()
            if not port:
                sys.exit("[ERR] no port entered")
        baud = input(f"Baud rate [{BAUD_DEFAULT}]: ").strip()
        return "usb", port, int(baud) if baud else BAUD_DEFAULT
    if ch in ("2", "3"):
        ip = input("Adapter IP: ").strip()
        if not ip:
            sys.exit("[ERR] no IP entered")
        return ("udp" if ch == "2" else "tcp"), ip, None
    sys.exit("[ERR] invalid choice")


def interactive_out():
    print("\nOutput sink:")
    print("  1. stdout (pipe/redirect to another program)")
    print("  2. virtual serial port (com0com, for serial software such as wfview)")
    print("  3. TCP server (local port, other programs connect)")
    print("  4. UDP forward (local port)")
    print("  5. file")
    ch = input("Select (1-5): ").strip()
    if ch == "1" or not ch:
        return "stdout"
    if ch == "2":
        ports = _list_ports()
        if ports:
            print("Available ports (com0com ports look like CNCA0/CNCB0):")
            for i, p in enumerate(ports, 1):
                print(f"    {i}. {p}")
            sel = input(f"Select (1-{len(ports)}) or type a port name: ").strip()
            if sel.isdigit() and 1 <= int(sel) <= len(ports):
                return f"com:{ports[int(sel) - 1]}"
            return f"com:{sel}" if sel else sys.exit("[ERR] no port entered")
        port = input("No ports detected, type a com0com port name manually (e.g. CNCA0): ").strip()
        return f"com:{port}" if port else sys.exit("[ERR] no port entered")
    if ch == "3":
        p = input("Listen port [5200]: ").strip()
        return f"tcp:{int(p) if p else 5200}"
    if ch == "4":
        p = input("UDP port [5300]: ").strip()
        return f"udp:{int(p) if p else 5300}"
    if ch == "5":
        p = input("File path [spectrum.bin]: ").strip()
        return f"file:{p if p else 'spectrum.bin'}"
    sys.exit("[ERR] invalid choice")


# ============================================================
#  Main flow
# ============================================================
def main():
    ap = argparse.ArgumentParser(
        prog="remote_en.py",
        description="FTDX10 adapter remote data relay (GUI-less)",
        epilog="Example: python remote_en.py udp 192.168.1.100 | spectrum_program")
    ap.add_argument("mode", nargs="?", choices=["usb", "udp", "tcp"],
                    help="data source: usb|udp|tcp")
    ap.add_argument("addr", nargs="?",
                    help="address: COM port (usb) or adapter IP (udp/tcp)")
    ap.add_argument("--out", default="stdout",
                    help="output: stdout|com:COMx|tcp:PORT|udp:PORT|file:PATH (default stdout)")
    ap.add_argument("--baud", type=int, default=BAUD_DEFAULT,
                    help=f"USB serial baud rate (default {BAUD_DEFAULT})")
    ap.add_argument("--port", type=int, default=TCP_PORT_DEFAULT,
                    help=f"TCP source port (default {TCP_PORT_DEFAULT})")
    args = ap.parse_args()

    # no arguments → interactive mode
    if not args.mode or not args.addr:
        mode, addr, baud = interactive_source()
        out = interactive_out()
    else:
        mode, addr, out, baud = args.mode, args.addr, args.out, args.baud

    sink = build_sink(out)
    stop = threading.Event()

    if mode == "usb":
        src = threading.Thread(target=usb_source,
                               args=(addr, baud, sink, stop), daemon=True)
    elif mode == "udp":
        src = threading.Thread(target=udp_source,
                               args=(addr, sink, stop), daemon=True)
    else:
        src = threading.Thread(target=tcp_source,
                               args=(addr, args.port, sink, stop), daemon=True)
    src.start()

    try:
        while src.is_alive():
            time.sleep(0.2)
    except KeyboardInterrupt:
        print("\n[EXIT] stopping...")
    finally:
        stop.set()
        time.sleep(0.2)
        sink.close()
    print("[EXIT] done")


if __name__ == "__main__":
    main()
