import os
import sys

PLUGINS = []
PLUGINS_PATH = []
path = os.path.dirname(__file__)
for dir in os.listdir(path):
    d_path = os.path.dirname(__file__) + '/' + dir
    if os.path.isdir(d_path) and dir.startswith('plugin_'):
        PLUGINS.append(dir)
        PLUGINS_PATH.append(d_path)
