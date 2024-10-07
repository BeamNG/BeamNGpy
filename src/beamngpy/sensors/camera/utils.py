from __future__ import annotations

from typing import Dict, List, cast
from xml.dom import minidom
from xml.etree import ElementTree
from xml.etree.ElementTree import Element, SubElement

import numpy as np
from PIL import Image, ImageDraw, ImageFont

from beamngpy.logging import BNGValueError, create_warning
from beamngpy.types import Int3, StrDict


def extract_bounding_boxes(
    semantic_image: Image.Image, instance_image: Image.Image, classes: StrDict
) -> List[StrDict]:
    semantic_data = np.array(semantic_image)
    instance_data = np.array(instance_image)

    if semantic_data.shape != instance_data.shape:
        raise BNGValueError(
            "Error - The given semantic and instance annotation images have different resolutions."
        )

    bounding_boxes: Dict[int, StrDict] = {}
    issued_warning = False
    for y in range(semantic_data.shape[0]):
        for x in range(semantic_data.shape[1]):
            colour = instance_data[y, x]
            colour_key = colour[0] * 65536 + colour[1] * 256 + colour[2]
            if colour_key == 0:
                continue

            clazz = semantic_data[y, x]
            clazz_key = clazz[0] * 65536 + clazz[1] * 256 + clazz[2]
            if not issued_warning and clazz_key not in classes:
                create_warning(
                    f"The color ({clazz[0]}, {clazz[1]}, {clazz[2]}) was not found in the class mapping. This may mean that the annotation image is corrupted or that there is a bug in the annotation system."
                )
                issued_warning = True
            if classes.get(clazz_key, "BACKGROUND") == "BACKGROUND":
                continue

            if colour_key in bounding_boxes:
                entry = bounding_boxes[colour_key]
                box = entry["bbox"]
                box[0] = min(box[0], x)
                box[1] = min(box[1], y)
                box[2] = max(box[2], x)
                box[3] = max(box[3], y)
            else:
                entry = {
                    "bbox": [x, y, x, y],
                    "class": classes[clazz_key],
                    "color": [*colour],
                }
                bounding_boxes[colour_key] = entry

    box_list: List[StrDict] = []
    for _, v in bounding_boxes.items():
        box_list.append(v)
    return box_list


def draw_bounding_boxes(
    bounding_boxes: List[StrDict],
    colour: Image.Image,
    width: int = 3,
    font: str = "arial.ttf",
    font_size: int = 14,
) -> Image.Image:
    colour = colour.copy()
    draw = ImageDraw.Draw(colour)

    try:
        image_font = ImageFont.truetype(font=font, size=font_size)
    except OSError:
        image_font = ImageFont.load_default()

    for i, box in enumerate(bounding_boxes):
        box_colour = box["color"]
        box_colour = (box_colour[0], box_colour[1], box_colour[2])
        box_corners = box["bbox"]
        draw.rectangle(box_corners, outline=box_colour, width=width)

        text = "{}_{}".format(box["class"], i)
        text_pos = (box_corners[0], box_corners[3])
        text_anchor = "lt"

        if text_pos[1] > colour.size[1] * 0.9:
            text_pos = (box_corners[0], box_corners[1])
            text_anchor = "lb"

        draw.text(
            text_pos,
            text,
            fill="#FFFFFF",
            stroke_width=2,
            stroke_fill="#000000",
            font=image_font,
            anchor=text_anchor,
        )

    return colour


def export_bounding_boxes_xml(
    bounding_boxes: List[StrDict],
    folder: str | None = None,
    filename: str | None = None,
    path: str | None = None,
    database: str | None = None,
    size: Int3 | None = None,
) -> str:
    root = Element("annotation")

    if folder:
        folder_elem = SubElement(root, "folder")
        folder_elem.text = folder

    if filename:
        file_elem = SubElement(root, "filename")
        file_elem.text = filename

    if path:
        path_elem = SubElement(root, "path")
        path_elem.text = path

    if database:
        source = SubElement(root, "source")
        database_elem = SubElement(source, "database")
        database_elem.text = database

    if size:
        size_elem = SubElement(root, "size")
        width = SubElement(size_elem, "width")
        width.text = str(size[0])
        height = SubElement(size_elem, "height")
        height.text = str(size[1])
        depth = SubElement(size_elem, "depth")
        depth.text = str(size[2])

    segmented = SubElement(root, "segmented")
    segmented.text = "0"

    for i, bbox in enumerate(bounding_boxes):
        object_elem = SubElement(root, "object")
        name = SubElement(object_elem, "name")
        name.text = "{}_{}".format(bbox["class"], i)
        pose = SubElement(object_elem, "pose")
        pose.text = "Unspecified"
        truncated = SubElement(object_elem, "truncated")
        truncated.text = "0"
        difficult = SubElement(object_elem, "difficult")
        difficult.text = "0"
        bndbox = SubElement(object_elem, "bndbox")
        xmin = SubElement(bndbox, "xmin")
        xmin.text = str(bbox["bbox"][0])
        ymin = SubElement(bndbox, "ymin")
        ymin.text = str(bbox["bbox"][1])
        xmax = SubElement(bndbox, "xmax")
        xmax.text = str(bbox["bbox"][2])
        ymax = SubElement(bndbox, "ymax")
        ymax.text = str(bbox["bbox"][3])

    ret = ElementTree.tostring(root, "utf-8")
    ret = cast(minidom.Node, minidom.parseString(ret))
    return ret.toprettyxml(indent="  ")
