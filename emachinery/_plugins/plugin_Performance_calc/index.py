"""
example
"""

import streamlit as st
import numpy as np
import pandas as pd

# Imported by default
import st_interact as interact
import output_postProcessing.cplot as cplot

# Imported by user
import bandwidth_calc as BW_calc
import hitWall_calc as HWall_calc
import control_performance_calc as CPC

def main(d_sim, user_config):
    if 'user.bool_apply_WC_tunner_for_speed_loop' in d_sim:
        if d_sim['user.bool_apply_WC_tunner_for_speed_loop'] == False:
            st.title("Performance of TI Method")
        elif d_sim['user.bool_apply_WC_tunner_for_speed_loop'] == True:
            st.title("Performance of WCtuner")
        else:
            st.title("WHAT?")
        # Get cplot data as time series
        cplot_data = cplot.read_data(st.session_state.user_selected_motor)
        # 
        Dual_Loop_Bandwidth_index_data = BW_calc.cal_CLBW_VLBW(cplot_data, d_sim)
        Time_Performance_index_data    = CPC.overshoot_Ptime_Stime_calc(cplot_data, d_sim) 
        min_max_df                     = HWall_calc.min_max_performance_calc(cplot_data)

        # if( d_sim['user.bool_apply_sweeping_frequency_excitation'] == False):
        st.write("### Performance Index")
        Bandwidth_df = pd.DataFrame(Dual_Loop_Bandwidth_index_data, index=['实际值', '理论值'])
        st.table(Bandwidth_df)
        Time_Performance_df = pd.DataFrame(Time_Performance_index_data, index=['实际值', '理论值'])
        st.table(Time_Performance_df)

        st.write("### Min and Max Values")
        st.table(min_max_df)
    else:
        # Get cplot data as time series
        cplot_data = cplot.read_data(st.session_state.user_selected_motor)
        # 
        Dual_Loop_Bandwidth_index_data = BW_calc.cal_CLBW_VLBW(cplot_data, d_sim)
        
        st.write("### Performance Index")
        Bandwidth_df = pd.DataFrame(Dual_Loop_Bandwidth_index_data, index=['实际值', '理论值'])
        st.table(Bandwidth_df)
    # else:
    #     st.write("Closed Loop Bandwidth")

    interact.c_save_run_module(d_sim, user_config)
    interact.c_simulation_visual_module(d_sim)

    # st.write(d_sim)

    # print(cplot.read_data(st.session_state.user_selected_motor))
    # print(type(cplot.read_data(st.session_state.user_selected_motor)))
    
    # print(type(cplot.read_data(st.session_state.user_selected_motor)))
    # st.write(st.session_state)

