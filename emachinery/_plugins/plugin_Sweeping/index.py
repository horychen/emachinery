"""
example
"""

import streamlit as st

# Imported by default
import st_interact as interact
import output_postProcessing.cplot as cplot
import Sweeping2Bode_Bandwidth as Sweeping2BB

def main(d_sim, user_config):
    st.title("Sweeping For Speed and Current Loop")
    cplot_data = cplot.read_data(st.session_state.user_selected_motor)


    # print(user_config)
    # print(type(user_config))
    interact.c_save_run_module(d_sim, user_config)
    interact.c_simulation_visual_module(d_sim)
