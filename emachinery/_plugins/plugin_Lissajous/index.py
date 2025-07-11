"""
example
"""
import os  # 导入 os 模块
import streamlit as st
import numpy as np
import pandas as pd

# Imported by default
import st_interact as interact
import output_postProcessing.cplot as cplot
import matplotlib.pyplot as plt



def main(d_sim, user_config):
    # 在 Streamlit 中显示图形
    # Streamlit 标题

    st.title("Lissajous Figure Example")

    cplot_data = cplot.read_data(st.session_state.user_selected_motor)
    time = cplot_data['(*CTRL).timebase']  # 假设时间数据存储在 'time' 键中
    psi_alpha = cplot_data['ACM.psi_AB[0]']
    psi_beta = cplot_data['ACM.psi_AB[1]']
    hat_psi_alpha = cplot_data['FE.STA.psi_A[0]']
    hat_psi_beta = cplot_data['FE.STA.psi_A[1]']
    
    # 限定时间范围（例如，取时间在 0 到 5 秒之间的数据）
    time_start = 0.001
    time_end = 0.36
    mask = (time >= time_start) & (time <= time_end)
    psi_alpha = psi_alpha[mask]
    psi_beta = psi_beta[mask]
    hat_psi_alpha = hat_psi_alpha[mask]
    hat_psi_beta = hat_psi_beta[mask]
    # 绘制李萨如图
    fig, ax = plt.subplots(figsize=(6, 6))  # 设置图形大小为正方形
    ax.plot(psi_alpha, psi_beta, label=r"$\psi_{\rm A}$", linestyle='--', color='red', alpha=0.7, linewidth=2)
    ax.plot(hat_psi_alpha, hat_psi_beta, label=r"$\hat{\psi}_{\rm A}$", color='black', alpha=0.70, linewidth=2)
    # ax.plot(psi_alpha, psi_beta, alpha=0.7, linewidth=2)
    # ax.plot(hat_psi_alpha, hat_psi_beta, linestyle='--', color='red', alpha=0.70, linewidth=2)
    # ax.set_title("Lissajous Figure")
    ax.set_xlabel(r"$\alpha$-axis", fontsize=24)
    ax.set_ylabel(r"$\beta$-axis", fontsize=24)
    # 固定 x 和 y 轴范围
    ax.set_xlim(-0.1787132, 0.1666692)  # 固定 x 轴范围
    ax.set_ylim(-0.2338902, 0.1433262)  # 固定 y 轴范围
    # ax.legend(fontsize=18, loc='upper left')
    ax.grid(True)
    ax.set_aspect('equal', adjustable='box')  # 设置轴比例为正方形
    ax.tick_params(axis='both', which='major', labelsize=20)  # 主刻度字体大小
    ax.tick_params(axis='both', which='minor', labelsize=20)  # 次刻度字体大小

    # x_limits = ax.get_xlim()
    # y_limits = ax.get_ylim()
    # print(f"Recorded x-axis limits: {x_limits}")
    # print(f"Recorded y-axis limits: {y_limits}")
    output_dir = os.path.join(os.path.dirname(__file__), "output")  # 构造相对路径
    os.makedirs(output_dir, exist_ok=True)  # 如果目录不存在，则创建
    output_path = os.path.join(output_dir, "lissajous_figure.png")  # 图片保存路径
    plt.savefig(output_path, dpi=300, bbox_inches='tight', transparent=True)  # 保存图片

    st.pyplot(fig)


