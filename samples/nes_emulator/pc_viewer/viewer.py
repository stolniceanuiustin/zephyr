#!/usr/bin/env python3
"""
NES framebuffer viewer (TCP).
Connects to the ZedBoard, receives 256x240 ARGB32 frames over TCP, displays them.

Protocol (matches the Zephyr sender in src/fb_udp.c):
  The board is a TCP server on PORT. For each frame it sends:
    [4 bytes]  length    uint32 big-endian  - number of payload bytes to follow
    [length]   payload   raw ARGB32 pixel data (256*240*4 bytes)
TCP guarantees order and delivery, so frames are never partial/dropped.
"""

import socket
import struct
import numpy as np
import pygame

WIDTH, HEIGHT = 256, 240
SCALE = 3
PORT = 5555
BOARD_IP = "192.0.2.3"
FRAME_BYTES = WIDTH * HEIGHT * 4


def recv_exact(sock, n):
    """Read exactly n bytes from the socket, or raise on disconnect."""
    buf = bytearray(n)
    view = memoryview(buf)
    got = 0
    while got < n:
        r = sock.recv_into(view[got:], n - got)
        if r == 0:
            raise ConnectionError("board closed connection")
        got += r
    return buf


def connect():
    """Block until the board accepts a connection."""
    while True:
        try:
            s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            s.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
            s.settimeout(2.0)
            s.connect((BOARD_IP, PORT))
            s.settimeout(None)
            print(f"Connected to {BOARD_IP}:{PORT}")
            return s
        except (OSError, socket.timeout):
            print(f"Waiting for board at {BOARD_IP}:{PORT}...")
            try:
                s.close()
            except OSError:
                pass


def main():
    pygame.init()
    screen = pygame.display.set_mode((WIDTH * SCALE, HEIGHT * SCALE))
    pygame.display.set_caption("NES Framebuffer (TCP)")
    clock = pygame.time.Clock()

    fb = np.zeros((HEIGHT, WIDTH, 3), dtype=np.uint8)
    sock = connect()

    running = True
    while running:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
            if event.type == pygame.KEYDOWN and event.key == pygame.K_ESCAPE:
                running = False

        try:
            hdr = recv_exact(sock, 4)
            (length,) = struct.unpack(">I", hdr)
            if length != FRAME_BYTES:
                # Out of sync / unexpected frame size — drop and reconnect.
                raise ConnectionError(f"unexpected frame length {length}")

            raw = recv_exact(sock, length)
            # Firmware palette is 0xRRGGBBAA; little-endian board lays the uint32
            # in memory as [A, B, G, R], so R G B are at indices 3, 2, 1.
            arr = np.frombuffer(bytes(raw), dtype=np.uint8).reshape(-1, 4)
            rgb = arr[:, [3, 2, 1]]
            fb = rgb.reshape(HEIGHT, WIDTH, 3)

        except (ConnectionError, OSError) as e:
            print(f"Connection lost ({e}); reconnecting...")
            try:
                sock.close()
            except OSError:
                pass
            sock = connect()
            continue

        surf = pygame.surfarray.make_surface(fb.swapaxes(0, 1))
        scaled = pygame.transform.scale(surf, (WIDTH * SCALE, HEIGHT * SCALE))
        screen.blit(scaled, (0, 0))
        pygame.display.flip()
        clock.tick(60)

    sock.close()
    pygame.quit()


if __name__ == "__main__":
    main()
