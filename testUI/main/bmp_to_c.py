#!/usr/bin/env python3
import sys

from PIL import Image

img = Image.open(sys.argv[1]).convert("1")  # 1-bit
w, h = img.size
print(f"// {w}x{h} px")
print(f"#define BMP_W {w}")
print(f"#define BMP_H {h}")
print("static const uint8_t bmp_data[] = {")
pix = img.load()
row_bytes = (w + 7) // 8
for y in range(h):
    row = []
    for bx in range(row_bytes):
        byte = 0
        for bit in range(8):
            x = bx * 8 + bit
            if x < w:
                # PIL: 0=czarny 255=biały → e-Paper: 0=czarny 1=biały (bit=1)
                if pix[x, y] != 0:
                    byte |= 0x80 >> bit
            else:
                byte |= 0x80 >> bit  # padding = biały
        row.append(f"0x{byte:02X}")
    print("    " + ",".join(row) + ",")
print("};")
