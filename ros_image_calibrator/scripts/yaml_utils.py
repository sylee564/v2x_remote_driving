#!/usr/bin/env python
# -*- coding: utf-8 -*-
import os, yaml

def ensure_dir(p):
    if not os.path.exists(p):
        os.makedirs(p)

def save_yaml(path, data):
    ensure_dir(os.path.dirname(path))
    with open(path, 'w') as f:
        yaml.safe_dump(data, f, default_flow_style=False, sort_keys=False)
