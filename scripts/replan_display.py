import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Line3DCollection
from pathlib import Path
from replan_timeparam_utils import pad_trajectories, load_traj_txt

def plot_3d_trajectories(master_pos, slave_pos, xyz_min=None, xyz_max=None, master_ref_pos=None):
    # 创建 3D 图形
    fig = plt.figure(figsize=(8, 6))
    ax = fig.add_subplot(111, projection='3d')

    # 绘制轨迹
    ax.plot(master_pos[:, 0], master_pos[:, 1], master_pos[:, 2], label='Master', marker='o')
    ax.plot(slave_pos[:, 0],  slave_pos[:, 1],  slave_pos[:, 2],  label='Slave',  marker='^')
    if master_ref_pos is not None:
        ax.scatter(master_ref_pos[:, 0], master_ref_pos[:, 1], master_ref_pos[:, 2], label='Master Path', color='red', alpha=0.5, marker='o')

    # —— 在同一时间戳连线 ——
    segments = np.stack([master_pos, slave_pos], axis=1)  # 形状: (N, 2, 3)
    lc = Line3DCollection(segments, linewidths=0.8, alpha=0.7)
    ax.add_collection3d(lc)

    # 保障可见范围（把显示体设为正方体，三轴单位长度一致）
    pts = np.vstack([master_pos, slave_pos])
    if xyz_min is None or xyz_max is None:
        xyz_min, xyz_max = pts.min(axis=0), pts.max(axis=0)
    # xyz_min, xyz_max = pts.min(axis=0), pts.max(axis=0)

    center = (xyz_min + xyz_max) / 2.0
    max_range = (xyz_max - xyz_min).max()
    r = max_range / 2.0
    pad = 0.02 * max_range  # 可选：给一点边距

    ax.set_xlim(center[0] - r - pad, center[0] + r + pad)
    ax.set_ylim(center[1] - r - pad, center[1] + r + pad)
    ax.set_zlim(center[2] - r - pad, center[2] + r + pad)
    ax.set_box_aspect((1, 1, 1))  # 均匀比例

    # 保障可见范围（3D Collection 不触发自动缩放）
    # pts = np.vstack([master_pos, slave_pos])
    # xyz_min, xyz_max = pts.min(axis=0), pts.max(axis=0)
    # ax.set_xlim(xyz_min[0], xyz_max[0])
    # ax.set_ylim(xyz_min[1], xyz_max[1])
    # ax.set_zlim(xyz_min[2], xyz_max[2])
    # ax.set_box_aspect((1, 1, 1))  # 等比例

    ax.set_xlabel('X [m]')
    ax.set_ylabel('Y [m]')
    ax.set_zlabel('Z [m]')
    ax.set_title('3D Trajectories of Master and Slave (with per-timestamp links)')
    ax.legend()
    plt.tight_layout()

    # 保存图片（与 master_path 同目录、同名 .png）
    # out_img = Path(master_path).with_suffix('.png')
    # fig.savefig(out_img, dpi=300, bbox_inches='tight')
    # print(f"[OK] Figure saved -> {out_img}")

    plt.show()
    # plt.close(fig)

if __name__ == "__main__":
    # 文件路径（根据实际路径修改）
    # master_path = 'data_trajectory_points/trans_scene1_09/clip6_stage_4_tcp_trajectory_4_densified_smoothed.txt'
    # master_path = 'data_trajectory_points/trans_scene1_09/clip6_stage_4_tcp_trajectory_4.txt'
    # master_path = 'data_trajectory_points/trans_scene1_11/clip8_stage_4_tcp_trajectory_4_densified_curvfix.txt'

    master_path = '/home/tp2/ws_humble/MTC_connect_visualization/leader_no_matching_tcp_001.txt'
    # slave_path  = 'data_trajectory_points/trans_scene1_09(slave)/clip6_stage_4_tcp_trajectory_4_densified_smoothed.txt'
    slave_path  = '/home/tp2/ws_humble/MTC_connect_visualization/follower_no_matching_tcp_001.txt'

    # slave_path  = 'data_trajectory_points/trans_scene1_09(slave)/clip6_stage_4_tcp_trajectory_4.txt'

    # 列名定义
    col_names = ['t'] + [f'c{i}' for i in range(16)]

    # 读取主从轨迹
    t_m, T_m = load_traj_txt(master_path)
    t_s, T_s = load_traj_txt(slave_path)

    # —— 对齐到“更长”的时间轴，并用最后一个样本填充短序列 ——
    t_m_a, T_m_a, t_s_a, T_s_a = pad_trajectories(t_m, T_m, t_s, T_s)

    # 提取位姿（c12, c13, c14 是末端平移位置）
    master_pos = T_m_a[:, :3, 3]
    slave_pos  = T_s_a[:, :3, 3]

    plot_3d_trajectories(master_pos, slave_pos)
