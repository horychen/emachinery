import os
import sys
import datetime
import yaml
import importlib
import streamlit as st
import st_interact as interact
import user_script_main as user
import _plugins

def streamlit_config():
    print('\n', '-='*30, datetime.datetime.now(), '=-'*30)
    st.set_page_config(layout="wide")
    # side bar appearance settings
    with st.sidebar:
        st.markdown(
            """
        <style>
        [data-testid="stSidebar"][aria-expanded="true"]{
            min-width: 450px;
            max-width: 1450px;
        }
        """,
            unsafe_allow_html=True,
        )
    return


def st_main_c(d_sim, user_config):
    """C main function"""
    st.title('⚙️Electric Machinery Simulation Visualization | C')
    interact.c_save_run_module(d_sim, user_config)
    interact.c_simulation_visual_module(d_sim)


def st_main_plugin(d_sim, user_config, user_selected_mode):
    """Plugins"""
    plugin_path = _plugins.PLUGINS_PATH[_plugins.PLUGINS.index(user_selected_mode)]
    sys.path = [path for path in sys.path if not any(path.startswith(p) for p in _plugins.PLUGINS_PATH)]
    sys.path.append(plugin_path)
    module_name = "index"
    spec = importlib.util.spec_from_file_location(module_name, os.path.join(plugin_path, module_name + ".py"))
    index = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(index)
    index.main(d_sim, user_config)

from rich import print
def main():
    st.session_state.DEFAULT_MOTOR_INDEX = 19
    st.session_state.DEFAULT_USER_INDEX = 5
    streamlit_config()

    user_selected_mode = interact.user_selected_mode()
    user_history = interact.get_user_history()
    user_history = interact.clear_history_moudle(user_history)

    with open(os.path.join(os.path.dirname(__file__), 'user_config.yaml'), encoding='utf-8') as f:
        user_config = yaml.load(f, Loader=yaml.FullLoader)

    with st.sidebar:
        with st.expander("仿真参数"):
            st.write(user_config['simulation'])

    d_sim = interact.option_select_motor(user_history, user_config)
    d_sim = interact.option_select_algorithm(d_sim, user_config)
    d_sim = user.user_pre_process(d_sim, user_config)
    interact.save_d_sim_2_dat_folder(d_sim)
    interact.online_para_editor(user_config['default_var_list'], d_sim)

    if user_selected_mode == 'C':
        st_main_c(d_sim, user_config)
    else:
        st_main_plugin(d_sim, user_config, user_selected_mode)

    interact.write_session_state(st.session_state)


if __name__ == '__main__':
    main()
