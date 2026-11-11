"""Render docs/ur3_gazebo_demo.txt (real armdoc output from a UR3 in Gazebo) as docs/ur3_gazebo.png."""
import os
from PIL import Image, ImageDraw, ImageFont

here = os.path.dirname(os.path.abspath(__file__))
lines = open(os.path.join(here, "ur3_gazebo_demo.txt")).read().rstrip("\n").split("\n")
font = ImageFont.truetype("/usr/share/fonts/truetype/dejavu/DejaVuSansMono.ttf", 18)
bold = ImageFont.truetype("/usr/share/fonts/truetype/dejavu/DejaVuSansMono-Bold.ttf", 18)
colors = {"ERROR": (255, 110, 110), "WARN": (240, 200, 90), "OK": (120, 220, 130), "$": (150, 190, 255)}
lh, pad, bar = 26, 22, 38
w = max(int(font.getlength(l)) for l in lines) + 2 * pad
img = Image.new("RGB", (max(w, 900), bar + pad * 2 + lh * len(lines)), (24, 26, 31))
d = ImageDraw.Draw(img)
d.rectangle([0, 0, img.width, bar], fill=(44, 47, 56))
for i, c in enumerate([(255, 95, 86), (255, 189, 46), (39, 201, 63)]):
    d.ellipse([16 + i * 24, 12, 28 + i * 24, 24], fill=c)
d.text((110, 9), "armdoc on a UR3 in Gazebo (ros2_control)", font=font, fill=(190, 195, 205))
y = bar + pad
for l in lines:
    key = "$" if l.startswith("$") else l.split(" ")[0]
    d.text((pad, y), l, font=bold if key != "$" else font, fill=colors.get(key, (215, 218, 225)))
    y += lh
img.save(os.path.join(here, "ur3_gazebo.png"))
