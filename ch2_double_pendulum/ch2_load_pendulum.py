# -*- coding: utf-8 -*-
"""
学习目标：
1. 学习如何用 Python API 从独立 XML 文件加载物理模型。
2. 掌握如何在代码中为模型设置初始状态（如设置关节的初始角度）。
3. 观察关节阻尼（damping）和摩擦力（frictionloss）对物理运动的影响。
"""

import os
import time
import mujoco
import mujoco.viewer

def main():
    # 获取 XML 文件的绝对路径
    current_dir = os.path.dirname(os.path.abspath(__file__))
    xml_path = os.path.join(current_dir, "ch2_double_pendulum.xml")
    
    print(f"正在从文件加载模型: {xml_path}")
    # 1. 使用 from_xml_path 从指定路径加载模型 (mjModel)
    model = mujoco.MjModel.from_xml_path(xml_path)
    # 2. 创建动态运行数据 (mjData)
    data = mujoco.MjData(model)
    
    print("模型加载成功！")
    print(f"模型关节数量 (nq): {model.nq}")  # nq 代表广义坐标的维度（即所有关节自由度的总和）
    
    # 3. 设置双摆的初始偏转角度，使其在重力下能够摆动
    # 重要原理：尽管我们在 XML 中声明了 angle="degree"，但在 Python API 内部，
    # 所有的角度物理量（data.qpos）在读取和写入时均使用“弧度（Radians）”作为标准单位。
    # 90 度 = 90 * pi / 180 ≈ 1.5708 弧度
    # 45 度 = 45 * pi / 180 ≈ 0.7854 弧度
    data.qpos[0] = 1.5708  # 设置第一个关节 (joint1) 的初始角度为 90 度
    data.qpos[1] = 0.7854  # 设置第二个关节 (joint2) 的初始角度为 45 度
  
    # 4. 启动被动渲染器
    print("正在拉起三维仿真窗口，请观察双摆在重力作用下的混沌摆动...")
    with mujoco.viewer.launch_passive(model, data) as viewer:
        
        while viewer.is_running():
            step_start = time.time()
            
            # 单步运行物理仿真
            mujoco.mj_step(model, data)
            
            # 同步渲染画面
            viewer.sync()
            
            # 维持运行速度与真实时间同步
            time_until_next_step = model.opt.timestep - (time.time() - step_start)
            if time_until_next_step > 0:
                time.sleep(time_until_next_step)
                
            # 每隔 1.0 秒打印一次两关节的实时角度（转换为度数显示）
            # 使用时间跨越边界判定，保证每秒只打印一次，避免重复打印
            if int(data.time) > int(data.time - model.opt.timestep):
                joint1_deg = data.qpos[0] * 180.0 / 3.1415926
                joint2_deg = data.qpos[1] * 180.0 / 3.1415926
                print(f"时间: {data.time:.1f}s | 关节1角度: {joint1_deg:6.1f}° | 关节2角度: {joint2_deg:6.1f}°")

if __name__ == "__main__":
    main()
