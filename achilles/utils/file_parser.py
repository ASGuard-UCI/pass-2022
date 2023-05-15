#!/usr/bin/env python

"""
Module used to provide parsers to common file types, e.g., yaml, json.
"""
import yaml
import json
import numpy as np
from PIL import Image

def parse_yaml(yaml_file):
    with open(yaml_file, 'r') as f:
        conf = yaml.load(f, Loader=yaml.FullLoader)
    return conf


def parse_json(json_file):
    with open(json_file, 'r') as f:
        dat = json.load(f)
    return dat


def read_image(img_file):
    with Image.open(img_file) as im:
        im.load()
        im_rgb = Image.new('RGB', im.size, (255, 255, 255))
        im_rgb.paste(im, mask=im.split()[3])
        img = np.array(im_rgb, dtype=np.uint8)
    return img
