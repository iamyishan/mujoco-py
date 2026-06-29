# -*- coding: utf-8 -*-
"""
学习目标：
1. 掌握 MuJoCo 仿真的核心循环机制 `mujoco.mj_step`。
2. 理解 mjData 中状态量存储的物理含义与区别：qpos, qvel, qacc 以及 xpos。
3. 学习自由关节 (free joint) 的状态表示，理解广义坐标维数 (nq) 与广义速度维数 (nv) 的不同（四元数表示）。
4. 学会利用 Python 读取实时状态，并使用 matplotlib 绘制物理量随时间变化的曲线。

运行方式：
在终端中执行：python ch3_free_drop.py
"""

import time
import os
import mujoco
import mujoco.viewer
import matplotlib.pyplot as plt
import numpy as np

# 1. 定义极简小球自由落体模型 (MJCF XML)
# 小球初始位置 pos="0 0 5.0" (位于 5 米高空)
# 使用 free 关节，使其拥有 6 个空间自由度
xml_content = """
<mujoco model="free_drop">
    <option gravity="0 0 -9.81" timestep="0.002"/>
    
    <asset>
        <!-- 灰色网格地面材质 -->
        <texture name="grid" type="2d" builtin="checker" rgb1=".2 .3 .4" rgb2=".1 .2 .3" width="512" height="512"/>
        <material name="grid_material" texture="grid" texrepeat="2 2" texuniform="true"/>
    </asset>

    <worldbody>
        <!-- 光源和地面 -->
        <light diffuse=".8 .8 .8" pos="0 0 10" dir="0 0 -1"/>
        <geom name="floor" type="plane" size="5 5 0.1" material="grid_material"/>
        
        <!-- 小球：质量为 1.5 kg，半径 0.2 m -->
        <body name="ball" pos="0 0 5.0">
            <joint name="free_ball" type="free"/>
            <geom name="ball_geom" type="sphere" size="0.2" rgba="0.8 0.4 0.2 1" mass="1.5"/>
        </body>
    </worldbody>
</mujoco>
"""

def main():
    # 2. 加载模型与运行数据
    model = mujoco.MjModel.from_xml_string(xml_content)
    data = mujoco.MjData(model)
    
    print("================== 模型参数与状态结构 ==================")
    # 理论要点：自由关节有 6 个自由度，但在 qpos 中用 7 个数表示 (3维位置 + 4维四元数姿态)
    # 在 qvel 和 qacc 中，速度和加速度只用 6 个数表示 (3维线速度 + 3维角速度)
    print(f"广义坐标位置维度 (model.nq): {model.nq}  --> qpos 包含 [x, y, z, qw, qx, qy, qz]")
    print(f"广义坐标速度维度 (model.nv): {model.nv}  --> qvel 包含 [vx, vy, vz, wx, wy, wz]")
    print(f"广义坐标加速度维度 (model.nv): {model.nv} --> qacc 包含 [ax, ay, az, alpha_x, alpha_y, alpha_z]")
    print(f"小球在 mjData.xpos 中世界系绝对坐标维度: {data.xpos.shape}")
    print("======================================================")

    # 3. 准备数据记录容器，用于后续绘图
    time_log = []
    z_pos_log = []      # qpos[2] 或 xpos[1, 2]
    z_vel_log = []      # qvel[2]
    z_acc_log = []      # qacc[2]

    # 设定模拟运行的总时长 (2秒)
    sim_duration = 2.0
    
    # 获取小球刚体的 ID (用于 xpos 的索引，0 是 worldbody, 1 是 ball)
    ball_body_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "ball")
    print(f"小球刚体的 ID: {ball_body_id}")

    print("启动物理仿真...")
    
    # 4. 启动被动渲染器
    with mujoco.viewer.launch_passive(model, data) as viewer:
        start_time = time.time()
        
        while viewer.is_running() and data.time < sim_duration:
            step_start = time.time()
            
            # (A) 记录当前步的状态
            time_log.append(data.time)
            
            # 方法一：从广义坐标 qpos / qvel / qacc 中提取 Z 轴状态
            # 因为只有一个自由关节，前 3 个数就是它的三维位置，第 2 个索引是 Z 轴位置
            z_pos_q = data.qpos[2]
            z_vel_q = data.qvel[2]
            z_acc_q = data.qacc[2]
            
            # 方法二：从全局笛卡尔坐标 xpos 中提取 Z 轴绝对位置
            # data.xpos[ball_body_id] 是小球在世界坐标系中的 [X, Y, Z] 坐标
            z_pos_x = data.xpos[ball_body_id][2]
            
            # 记录数据 (这里记录 qpos 提取的值，并把 xpos 提取的值做对比验证)
            z_pos_log.append(z_pos_q)
            z_vel_log.append(z_vel_q)
            z_acc_log.append(z_acc_q)
            
            # (B) 执行单步物理仿真计算
            # 计算重力、碰撞以及状态更新。计算后，qpos, qvel, qacc 等会被更新为下一步的值
            mujoco.mj_step(model, data)
            
            # (C) 同步渲染窗口
            viewer.sync()
            
            # (D) 控制运行速度，与现实时间同步
            time_until_next_step = model.opt.timestep - (time.time() - step_start)
            if time_until_next_step > 0:
                time.sleep(time_until_next_step)
                
            # 每隔 0.2 秒在控制台打印一次状态
            if int(data.time * 500) % 100 == 0:
                print(f"时间: {data.time:.3f}s | "
                      f"Z位置(qpos): {z_pos_q:.3f}m | "
                      f"Z位置(xpos): {z_pos_x:.3f}m | "
                      f"Z速度: {z_vel_q:.3f}m/s | "
                      f"Z加速度: {z_acc_q:.3f}m/s²")
                
    print("仿真结束，正在生成状态分析图表...")
    
    # 5. 使用 matplotlib 绘制仿真状态曲线
    # 创建 3 行 1 列的子图
    fig, axs = plt.subplots(3, 1, figsize=(10, 8), sharex=True)
    
    # Z轴位置曲线
    axs[0].plot(time_log, z_pos_log, label="Z Position (qpos)", color="red", linewidth=2)
    axs[0].set_ylabel("Height (m)", fontsize=11)
    axs[0].grid(True, linestyle="--", alpha=0.6)
    axs[0].legend(loc="upper right")
    axs[0].set_title("Chapter 3: Ball Free Drop State Analysis", fontsize=14, pad=15)
    
    # Z轴速度曲线
    axs[1].plot(time_log, z_vel_log, label="Z Velocity (qvel)", color="blue", linewidth=2)
    axs[1].set_ylabel("Velocity (m/s)", fontsize=11)
    axs[1].grid(True, linestyle="--", alpha=0.6)
    axs[1].legend(loc="lower right")
    
    # Z轴加速度曲线
    axs[2].plot(time_log, z_acc_log, label="Z Acceleration (qacc)", color="green", linewidth=2)
    axs[2].set_xlabel("Time (s)", fontsize=11)
    axs[2].set_ylabel("Acceleration (m/s²)", fontsize=11)
    axs[2].grid(True, linestyle="--", alpha=0.6)
    axs[2].legend(loc="lower right")
    
    # 自动调整布局并保存图片
    plt.tight_layout()
    plot_filename = "ch3_free_drop_plot.png"
    plt.savefig(plot_filename, dpi=300)
    print(f"图表已成功保存至本地: {os.path.abspath(plot_filename)}")
    
    # 显示图表
    plt.show()

if __name__ == "__main__":
    main()
