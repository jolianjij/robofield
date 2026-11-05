"""Render a fake top-down camera image of the field (1 px = 1 mm, no lens) for tests."""
import math

import cv2
import numpy as np

MARGIN = 100
FIELD = 1000
DICT = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)


def field_to_img(x, y):
    return MARGIN + x, MARGIN + (FIELD - y)   # image rows grow downward


def draw_marker(canvas, marker_id, x, y, size, theta):
    """Paste a marker centred at field (x, y) with its top edge pointing along theta."""
    px = 200
    tag = cv2.aruco.generateImageMarker(DICT, marker_id, px)
    tag = cv2.copyMakeBorder(tag, px // 6, px // 6, px // 6, px // 6, cv2.BORDER_CONSTANT, value=255)
    n = tag.shape[0]
    s_out = size * n / px                          # size including the white border
    fx, fy = math.cos(theta), math.sin(theta)      # forward (marker up)
    rx, ry = fy, -fx                               # marker right
    h = s_out / 2
    pts = [(-h, h), (h, h), (h, -h), (-h, -h)]     # TL, TR, BR, BL in (right, up)
    dst = [field_to_img(x + a * rx + b * fx, y + a * ry + b * fy) for a, b in pts]
    src = np.float32([[0, 0], [n, 0], [n, n], [0, n]])
    M = cv2.getPerspectiveTransform(src, np.float32(dst))
    warped = cv2.warpPerspective(tag, M, (canvas.shape[1], canvas.shape[0]), borderValue=0)
    mask = cv2.warpPerspective(np.full_like(tag, 255), M, (canvas.shape[1], canvas.shape[0]))
    canvas[mask > 0] = warped[mask > 0]


def render(robots, corner_size=60):
    canvas = np.full((FIELD + 2 * MARGIN, FIELD + 2 * MARGIN), 200, np.uint8)
    for i, (cx, cy) in {0: (0, 0), 1: (1000, 0), 2: (1000, 1000), 3: (0, 1000)}.items():
        draw_marker(canvas, i, cx, cy, corner_size, math.pi / 2)
    for mid, x, y, size, theta in robots:
        draw_marker(canvas, mid, x, y, size, theta)
    return canvas
