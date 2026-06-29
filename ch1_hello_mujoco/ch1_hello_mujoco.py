# -*- coding: utf-8 -*-
"""
学习目标：
1. 成功安装并导入官方 `mujoco` 库。
2. 了解 `mjModel` 与 `mjData` 的基本概念。
3. 学习如何用 Python 加载 XML 字符串格式的物理模型。
4. 使用 MuJoCo 自带的被动渲染器 (Passive Viewer) 开启一个三维仿真窗口。
"""

import time
import mujoco
import mujoco.viewer

# 定义一个极简的 MJCF (XML) 模型
# 该模型包含：一个光源、一个地面 (plane) 以及一个带有自由关节 (free joint) 的绿色小球 (sphere)
xml_content = """
<mujoco model="hello_world">
    <worldbody>
        <!-- 添加光源，使三维场景可见 -->
        <light diffuse=".5 .5 .5" pos="0 0 3" dir="0 0 -1"/>
        
        <!-- 定义地面：类型为平面，大小为 2x2 米，颜色为灰色 -->
        <geom name="ground" type="plane" size="2 2 0.1" rgba=".9 .9 .9 1"/>
        
        <!-- 定义一个小球刚体：初始位置在 z=2.0 米高处 -->
        <body name="ball" pos="0 0 2.0">
            <!-- 自由关节让该刚体在 3D 空间中拥有 6 个自由度 (可以自由下落、旋转) -->
            <joint type="free"/>
            <!-- 小球的几何外形：球体，半径 0.15 米，颜色为绿色，质量为 1 kg -->
            <geom name="ball_geom" type="sphere" size="0.15" rgba="0 .9 0 1" mass="1.0"/>
        </body>
    </worldbody>
</mujoco>
"""

def main():
    print("正在加载模型...")
    # 1. 从 XML 字符串加载模型结构 (mjModel)
    # mjModel 存储了模型的静态信息：物体的几何大小、质量、关节类型、重力大小等，这些在仿真过程中通常是不变的
    model = mujoco.MjModel.from_xml_string(xml_content)
    
    # 2. 创建模型对应的动态状态 (mjData)
    # mjData 存储了仿真的实时动态信息：如物体在当前时刻的位置(qpos)、速度(qvel)、受力情况等
    data = mujoco.MjData(model)
    
    print("模型加载成功！")
    print(f"当前仿真重力设置：{model.opt.gravity}")
    print(f"仿真单步时间步长 (timestep)：{model.opt.timestep} 秒")

    # 3. 启动 MuJoCo 的被动可视化窗口 (viewer.launch_passive)
    # 该窗口是异步运行的，不会阻塞我们的 Python 控制主循环
    print("正在拉起三维仿真窗口，请在弹出的窗口中观察小球下落...")
    with mujoco.viewer.launch_passive(model, data) as viewer:
        
        # 记录仿真运行的时间
        start_time = time.time()
        
        # 持续运行仿真
        while viewer.is_running():
            step_start = time.time()
            
            # 4. 执行物理引擎的单步计算 (mj_step)
            # 它会根据物理定律（重力、碰撞等），计算出下一时刻所有刚体的新位置和速度
            mujoco.mj_step(model, data)
            
            # 5. 将计算好的新物理状态同步到 3D 渲染器窗口中
            viewer.sync()
            
            # 6. 控制仿真运行速度与真实时间同步
            # 如果仿真计算太快，进行适当睡眠，防止画面过度加速
            time_until_next_step = model.opt.timestep - (time.time() - step_start)
            if time_until_next_step > 0:
                time.sleep(time_until_next_step)
                
            # 每隔约 0.1 秒在终端打印一下小球的实时 Z 轴高度
            # 通过判断仿真时间是否跨越了 0.1 秒的整数倍边界来触发打印，避免浮点数精度带来的漏打或多打
            if int(data.time * 10) > int((data.time - model.opt.timestep) * 10):
                print(f"仿真时间: {data.time:.2f}s | 小球高度 (Z): {data.qpos[2]:.4f}m")

if __name__ == "__main__":
    main()
