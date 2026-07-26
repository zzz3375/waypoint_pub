#!/usr/bin/env python3
"""Persistent serial-TCP bridge for PX4.
Keeps serial port open across TCP client disconnects/reconnects.
"""

import serial
import socket
import select
import sys
import signal
import time

SERIAL_DEV = "/dev/serial/by-id/usb-Holybro_PX4_KakuteH7_0-if00"
BAUD = 921600
TCP_PORT = 5760
BIND_HOST = "0.0.0.0"

running = True

def shutdown(sig, frame):
    global running
    print(f"\n[{time.strftime('%H:%M:%S')}] Shutting down...")
    running = False

signal.signal(signal.SIGINT, shutdown)
signal.signal(signal.SIGTERM, shutdown)

def main():
    # Open serial once — never close it
    print(f"[{time.strftime('%H:%M:%S')}] Opening serial: {SERIAL_DEV} @ {BAUD}")
    ser = serial.Serial(SERIAL_DEV, BAUD, timeout=0)

    # Create listening socket
    server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    server.bind((BIND_HOST, TCP_PORT))
    server.listen(1)
    server.setblocking(False)
    print(f"[{time.strftime('%H:%M:%S')}] Listening on TCP {BIND_HOST}:{TCP_PORT}")

    client = None

    while running:
        # Build fd list for select
        rlist = [server]
        if client:
            rlist.append(client)
        rlist.append(ser)

        try:
            readable, _, _ = select.select(rlist, [], [], 1.0)
        except (select.error, ValueError):
            continue

        for fd in readable:
            if fd is server:
                # New connection
                conn, addr = server.accept()
                if client:
                    print(f"[{time.strftime('%H:%M:%S')}] New client {addr}, closing previous")
                    client.close()
                client = conn
                client.setblocking(False)
                print(f"[{time.strftime('%H:%M:%S')}] Client connected: {addr}")

            elif fd is client and client:
                # Data from TCP client → serial
                try:
                    data = client.recv(4096)
                    if data:
                        ser.write(data)
                    else:
                        print(f"[{time.strftime('%H:%M:%S')}] Client disconnected")
                        client.close()
                        client = None
                except (ConnectionResetError, BrokenPipeError, OSError):
                    print(f"[{time.strftime('%H:%M:%S')}] Client connection lost")
                    client.close()
                    client = None

            elif fd is ser:
                # Data from serial → TCP client
                try:
                    data = ser.read(4096)
                    if data and client:
                        try:
                            client.sendall(data)
                        except (ConnectionResetError, BrokenPipeError, OSError):
                            print(f"[{time.strftime('%H:%M:%S')}] Send failed, client gone")
                            client.close()
                            client = None
                except serial.SerialException as e:
                    print(f"[{time.strftime('%H:%M:%S')}] Serial error: {e}", file=sys.stderr)
                    time.sleep(0.5)

    # Cleanup
    if client:
        client.close()
    ser.close()
    server.close()
    print(f"[{time.strftime('%H:%M:%S')}] Exited.")

if __name__ == "__main__":
    main()
