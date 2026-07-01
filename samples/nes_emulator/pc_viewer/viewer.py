#!/usr/bin/env python3
"""
NES framebuffer viewer (TCP).
Connects to the ZedBoard, receives 256x240 palette-index frames over TCP and
resolves them to RGB locally for display.

Protocol (matches the Zephyr sender in src/fb_udp.c):
  The board is a TCP server on PORT. For each frame it sends:
    [4 bytes]  length    uint32 big-endian  - number of payload bytes to follow
    [length]   payload   one byte per pixel (6-bit NES palette index), 256*240 bytes
TCP guarantees order and delivery, so frames are never partial/dropped.

The board sends palette indices (1 byte/pixel) instead of resolved ARGB32
(4 bytes/pixel) to cut the per-frame payload by 4x. The palette->RGB lookup
lives here and must mirror nes_pallete_32 in include/virtual_screen.h.
"""

import socket
import struct
import numpy as np
import pygame

WIDTH, HEIGHT = 256, 240
SCALE = 3
PORT = 5555
BOARD_IP = "192.0.2.3"
FRAME_BYTES = WIDTH * HEIGHT  # 1 byte/pixel (palette index)

# NES palette, RGB. Mirrors nes_pallete_32 (0xRRGGBBAA) in
# include/virtual_screen.h — keep the two in sync.
NES_PALETTE = np.array([
    (0x7C, 0x7C, 0x7C), (0x00, 0x00, 0xFC), (0x00, 0x00, 0xBC), (0x44, 0x28, 0xBC),
    (0x94, 0x00, 0x84), (0xA8, 0x00, 0x20), (0xA8, 0x10, 0x00), (0x88, 0x14, 0x00),
    (0x50, 0x30, 0x00), (0x00, 0x78, 0x00), (0x00, 0x68, 0x00), (0x00, 0x58, 0x00),
    (0x00, 0x40, 0x58), (0x00, 0x00, 0x00), (0x00, 0x00, 0x00), (0x00, 0x00, 0x00),
    (0xBC, 0xBC, 0xBC), (0x00, 0x78, 0xF8), (0x00, 0x58, 0xF8), (0x68, 0x44, 0xFC),
    (0xD8, 0x00, 0xCC), (0xE4, 0x00, 0x58), (0xF8, 0x38, 0x00), (0xE4, 0x5C, 0x10),
    (0xAC, 0x7C, 0x00), (0x00, 0xB8, 0x00), (0x00, 0xA8, 0x00), (0x00, 0xA8, 0x44),
    (0x00, 0x88, 0x88), (0x00, 0x00, 0x00), (0x00, 0x00, 0x00), (0x00, 0x00, 0x00),
    (0xF8, 0xF8, 0xF8), (0x3C, 0xBC, 0xFC), (0x68, 0x88, 0xFC), (0x98, 0x78, 0xF8),
    (0xF8, 0x78, 0xF8), (0xF8, 0x58, 0x98), (0xF8, 0x78, 0x58), (0xFC, 0xA0, 0x44),
    (0xF8, 0xB8, 0x00), (0xB8, 0xF8, 0x18), (0x58, 0xD8, 0x54), (0x58, 0xF8, 0x98),
    (0x00, 0xE8, 0xD8), (0x78, 0x78, 0x78), (0x00, 0x00, 0x00), (0x00, 0x00, 0x00),
    (0xFC, 0xFC, 0xFC), (0xA4, 0xE4, 0xFC), (0xB8, 0xB8, 0xF8), (0xD8, 0xB8, 0xF8),
    (0xF8, 0xB8, 0xF8), (0xF8, 0xA4, 0xC0), (0xF0, 0xD0, 0xB0), (0xFC, 0xE0, 0xA8),
    (0xF8, 0xD8, 0x78), (0xD8, 0xF8, 0x78), (0xB8, 0xF8, 0xB8), (0xB8, 0xF8, 0xD8),
    (0x00, 0xFC, 0xFC), (0xF8, 0xD8, 0xF8), (0x00, 0x00, 0x00), (0x00, 0x00, 0x00),
], dtype=np.uint8)


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
            # One byte per pixel: a 6-bit NES palette index. Mask to 63 (defensive
            # against stray high bits) and look up RGB via the palette LUT.
            idx = np.frombuffer(bytes(raw), dtype=np.uint8) & 0x3F
            fb = NES_PALETTE[idx].reshape(HEIGHT, WIDTH, 3)

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
