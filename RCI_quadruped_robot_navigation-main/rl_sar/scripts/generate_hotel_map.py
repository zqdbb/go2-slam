#!/usr/bin/env python3
"""Generate a ROS occupancy map from the hotel wall collision meshes."""

import math
import pathlib
import xml.etree.ElementTree as ET

from PIL import Image, ImageDraw
import yaml


RESOLUTION = 0.05
ORIGIN_X = 0.0
ORIGIN_Y = -50.0
WIDTH_M = 60.0
HEIGHT_M = 60.0


def world_to_pixel(x, y, height_px):
    px = round((x - ORIGIN_X) / RESOLUTION)
    py = height_px - 1 - round((y - ORIGIN_Y) / RESOLUTION)
    return px, py


def load_obj(path):
    vertices = []
    faces = []
    for line in path.read_text(encoding="utf-8", errors="ignore").splitlines():
        fields = line.split()
        if not fields:
            continue
        if fields[0] == "v" and len(fields) >= 4:
            vertices.append(tuple(float(v) for v in fields[1:4]))
        elif fields[0] == "f" and len(fields) >= 4:
            faces.append([int(v.split("/")[0]) - 1 for v in fields[1:]])
    return vertices, faces


def main():
    package_dir = pathlib.Path(__file__).resolve().parents[1]
    model_dir = package_dir / "models" / "hotel_L1"
    root = ET.parse(model_dir / "model.sdf").getroot()
    width_px = round(WIDTH_M / RESOLUTION)
    height_px = round(HEIGHT_M / RESOLUTION)
    image = Image.new("L", (width_px, height_px), 0)
    draw = ImageDraw.Draw(image)

    links = root.findall(".//link")
    for prefix, fill, line_width in (("floor_", 254, 1), ("wall_", 0, 3)):
        for link in links:
            if not link.attrib.get("name", "").startswith(prefix):
                continue
            pose = [float(v) for v in (link.findtext("pose") or "0 0 0 0 0 0").split()]
            tx, ty, _, _, _, yaw = pose
            uri = link.findtext("collision/geometry/mesh/uri")
            if not uri:
                continue
            vertices, faces = load_obj(model_dir / "meshes" / pathlib.Path(uri).name)
            c, s = math.cos(yaw), math.sin(yaw)
            transformed = [
                (tx + c * x - s * y, ty + s * x + c * y, z)
                for x, y, z in vertices
            ]
            for face in faces:
                points = [world_to_pixel(transformed[i][0], transformed[i][1], height_px) for i in face]
                draw.polygon(points, fill=fill)
                draw.line(points + [points[0]], fill=fill, width=line_width)

    project_dir = package_dir.parent
    for name in ("elevator_lift1.yaml", "elevator_lift2.yaml"):
        config_path = project_dir / "bt" / "elevator_bt" / "config" / name
        config = yaml.safe_load(config_path.read_text(encoding="utf-8"))
        zones = config["zones"]
        cabin_min = zones["cabin_min"]
        cabin_max = zones["cabin_max"]
        zone_min = zones["zone_min"]
        zone_max = zones["zone_max"]
        draw.rectangle(
            [
                world_to_pixel(cabin_min[0], cabin_max[1], height_px),
                world_to_pixel(cabin_max[0], cabin_min[1], height_px),
            ],
            fill=254,
        )
        center_x = 0.5 * (cabin_min[0] + cabin_max[0])
        gap_y_min = min(cabin_max[1], zone_max[1])
        gap_y_max = max(cabin_min[1], zone_min[1])
        draw.rectangle(
            [
                world_to_pixel(center_x - 0.55, gap_y_max, height_px),
                world_to_pixel(center_x + 0.55, gap_y_min, height_px),
            ],
            fill=254,
        )

    output_dir = package_dir / "maps"
    output_dir.mkdir(exist_ok=True)
    image.save(output_dir / "hotel_l1.pgm")
    print(output_dir / "hotel_l1.pgm")


if __name__ == "__main__":
    main()
