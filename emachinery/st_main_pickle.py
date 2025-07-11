import os
import sys
import datetime
import yaml
import importlib
import streamlit as st
import st_interact as interact
import user_script_main as user
import _plugins

import pickle
if __name__ == "__main__":
    with open(os.path.dirname(__file__)+"/_plugins/plugin_MOO/data/pop.pkl", 'rb') as pickle_file:
        pop = pickle.load(pickle_file)
        print(pop)
        # pop.evolve()
