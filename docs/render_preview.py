"""Render a 240x320 preview that mirrors Apps/display layout & 5x5 font.

Run:  python docs/render_preview.py
Out:  docs/ui_preview.png / ui_preview_2x.png        (heading calibrated)
      docs/ui_preview_uncal_2x.png                   (heading not calibrated)
"""
import os
import re
from PIL import Image

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
SRC = os.path.join(ROOT, "Apps", "display", "display_text.c")
src = open(SRC, encoding="utf-8").read()
rows = re.findall(r"\{([^}]*)\}", src)
glyphs = []
for r in rows:
    vals = [int(x, 16) for x in re.findall(r"0x([0-9a-fA-F]{2})", r)]
    if len(vals) == 6:
        glyphs.append(vals)
assert len(glyphs) == 96, len(glyphs)

def rgb565(r, g, b):
    return (((r & 0xF8) << 8) | ((g & 0xFC) << 3) | ((b & 0xF8) >> 3))

def to888(c):
    r = (c >> 11) & 0x1F
    g = (c >> 5) & 0x3F
    b = c & 0x1F
    return (r * 255 // 31, g * 255 // 63, b * 255 // 31)

BG      = to888(rgb565(8, 12, 18))
HEADER  = to888(rgb565(16, 24, 32))
ACCENT  = to888(rgb565(0, 200, 255))
TEXT    = to888(rgb565(235, 241, 247))
LABEL   = to888(rgb565(122, 138, 153))
DATE    = to888(rgb565(176, 196, 212))
DIVIDER = to888(rgb565(42, 58, 74))
FIX     = to888(rgb565(14, 90, 42))
WARN    = to888(rgb565(255, 176, 32))

W, H = 240, 320

def text_width(s, scale):
    return len(s) * 6 * scale - scale if s else 0

def draw(heading_text, heading_fg):
    """Draw the whole panel; the only varying part is the heading field."""
    img = Image.new("RGB", (W, H), BG)
    px = img.load()

    def rect(x, y, w, h, color):
        for yy in range(max(0, y), min(H, y + h)):
            for xx in range(max(0, x), min(W, x + w)):
                px[xx, yy] = color

    def render(x, y, w, h, s, scale, fg, bg, align="left"):
        rect(x, y, w, h, bg)
        if not s:
            return
        tw = text_width(s, scale)
        if align == "center":
            x0 = (w - tw) // 2
        elif align == "right":
            x0 = w - tw
        else:
            x0 = 0
        y0 = (h - 5 * scale) // 2
        for n, ch in enumerate(s):
            g = glyphs[ord(ch) - 0x20]
            cx = x0 + n * 6 * scale
            for col in range(5):
                for row in range(5):
                    if g[col] & (1 << (row + 2)):
                        rect(x + cx + col * scale, y + y0 + row * scale,
                             scale, scale, fg)

    # chrome
    rect(0, 0, W, H, BG)
    rect(0, 0, W, 32, HEADER)
    render(10, 11, text_width("HARDBRICK", 2), 10, "HARDBRICK", 2, TEXT, HEADER)
    render(182, 11, text_width("GNSS", 2), 10, "GNSS", 2, ACCENT, HEADER)
    rect(0, 32, W, 2, ACCENT)
    rect(12, 96, 216, 1, DIVIDER)
    rect(12, 240, 216, 1, DIVIDER)
    for label, yy in (("LAT", 108), ("LON", 148), ("ALT", 195), ("SAT", 223)):
        render(12, yy, text_width(label, 2), 10, label, 2, LABEL, BG)

    # dynamic fields
    render(0, 46, 240, 15, "12:34:56", 3, TEXT, BG, "center")
    render(0, 72, 240, 10, "2026-10-24", 2, DATE, BG, "center")
    render(12, 122, 216, 15, "31.230416 N", 3, TEXT, BG)
    render(12, 162, 216, 15, "121.473701 E", 3, TEXT, BG)
    render(56, 190, 172, 15, "12.3 m", 3, TEXT, BG)
    render(56, 218, 172, 15, "08", 3, TEXT, BG)
    rect(12, 250, 216, 40, FIX)
    render(12, 265, 216, 10, "FIX SINGLE", 2, TEXT, FIX, "center")

    # heading row (display.c: HDG_DIV_Y / HDG_Y / HDG_LABEL_Y / HDG_VALUE_*)
    rect(12, 291, 216, 1, DIVIDER)
    render(12, 292 + (28 - 10) // 2, text_width("HDG", 2), 10, "HDG", 2, LABEL, BG)
    render(112, 292 + (28 - 15) // 2, 116, 15, heading_text, 3,
           heading_fg, BG, "right")

    return img

def save(img, name, scale=1):
    out = img.resize((W * scale, H * scale), Image.NEAREST) if scale > 1 else img
    out.save(os.path.join(HERE, name))

base = draw("123", ACCENT)
save(base, "ui_preview.png")
save(base, "ui_preview_2x.png", 2)
save(draw("UNCAL", WARN), "ui_preview_uncal_2x.png", 2)
print("saved")
