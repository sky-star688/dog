#!/usr/bin/env python3
"""OpenMV color-target demo (runs on the OpenMV camera).

Streams ``@OVM:<cx>,<cy>,<width>,<height>,<pixels>!`` frames over USB VCP when
a colour blob is found.  This mirrors the host-side protocol in
``lite3_inspection.openmv``; it never drives the arm.

Edit the thresholds for the target colour before flashing.
"""

import sensor
import time
from pyb import USB_VCP

# Colour thresholds (L, A, B min/max) — calibrate for the actual target.
THRESHOLDS = [(30, 100, 15, 127, 15, 127)]
MIN_PIXELS = 200

sensor.reset()
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.QVGA)
sensor.skip_frames(time=1500)

usb = USB_VCP()
clock = time.clock()

while True:
    clock.tick()
    img = sensor.snapshot()
    blobs = img.find_blobs(THRESHOLDS, pixels_threshold=MIN_PIXELS)

    if blobs:
        blob = max(blobs, key=lambda item: item.pixels())
        frame = "@OVM:%d,%d,%d,%d,%d!" % (
            blob.cx(),
            blob.cy(),
            blob.w(),
            blob.h(),
            blob.pixels(),
        )
        usb.send(frame.encode("ascii"))
