#!/bin/env python3
#
# This test cut differs from the test cut in Silhouette Studio:
# as our triangle is not a simple triangle, but a more complex
# Arrow head. The outer square is an ordinary 1cm x 1cm square.
#

import sys, os
from svgpathtools import svg2paths, Path, Line

sys.path.append('/usr/share/inkscape/extensions')
sys.path.append(os.path.expanduser('~/.config/inkscape/extensions/'))
from silhouette.Graphtec import SilhouetteCameo


def get_paths(filename):
    paths: list[Path] = svg2paths(filename)[0]
    contours = []

    for path in paths:
        contour = []
        num_segments = len(path)

        for i in range(num_segments):
            z = path.point(i / (num_segments - 1))
            contour.append((z.real, z.imag))

        contours.append(contour)

    return contours


def scale(contours: list[list[tuple[float, float]]], max_width: float, max_height: float) -> list[list[(int, int)]]:
    min_x = float('inf')
    max_x = float('-inf')
    min_y = float('inf')
    max_y = float('-inf')
    for p in contours:
        for x, y in p:
            min_x = min(min_x, x)
            max_x = max(max_x, x)

            min_y = min(min_y, y)
            max_y = max(max_y, y)

    w = max_x - min_x
    h = max_y - min_y

    factor_width = max_width / w
    factor_height = max_height / h

    if factor_width > factor_height:
        factor_height = factor_width
    else:
        factor_width = factor_height

    scaled = []
    for p in contours:
        path = []
        for x, y in p:
            x = (x - min_x) * factor_width
            y = (y - min_y) * factor_height

            path.append((x, y))

        scaled.append(path)

    return scaled


def write_progress(done, total, msg):
    perc = 100. * done / total
    print("%d%% %s\r" % (perc, msg))


file_name = "../pipeline/img.svg"
contours = get_paths(file_name)
print(contours)

contours = scale(contours, max_width=100, max_height=100)
print(contours)
# exit()

dev = SilhouetteCameo(progress_cb=write_progress, dry_run=False)
state = dev.status()  # hint at loading paper, if not ready.
print(f'status={state}')
print("device version: '%s'" % dev.get_version())

dev.setup(media=132, pen=True, pressure=10, speed=5)
dev.plot(pathlist=contours, offset=(15, 15))
