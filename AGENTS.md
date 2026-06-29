# AGENTS.md

## 1. 项目基础信息
- 项目类型：MuJoCo机器人强化学习仿真入门项目
- 核心技术栈：mujoco 3.10.0+ python 3.10+强化学习
- 项目用途：供机器人仿真初学者学习，可满足入门到精通

## 2. 项目目录结构规范
项目目录分为三级：
- 一级：项目根目录 `mujoco-py`
- 二级：章节名称文件夹（如 `ch1_hello_mujoco/`）
- 三级：各章节的具体文件（如 `.py`, `.xml`, `.md` 等）

### 目录图谱预览：
```text
mujoco-py/                              # 一级根目录
├── AGENTS.md                           # 本项目说明与规范文件
├── mujoco_course_plan.md               # 课程大纲与整体学习计划
├── ch1_hello_mujoco/                   # 二级目录：第一章
│   └── ch1_hello_mujoco.py             # 三级文件
├── ch2_double_pendulum/                # 二级目录：第二章
│   ├── ch2_double_pendulum.xml         # 三级文件
│   └── ch2_load_pendulum.py            # 三级文件
└── ch3_free_drop/                      # 二级目录：第三章
    ├── ch3_free_drop.py                # 三级文件
    └── ch3_python_api_principles.md    # 三级文件
```

---

## 3. 文件整理记录
为保证项目整洁，现有文件已完成整理，你可以通过下面的链接直接访问：

| 文件 | 整理后新路径 | 说明 |
| :--- | :--- | :--- |
| **项目大纲** | [mujoco_course_plan.md](file:///d:/code/learning/mujoco-py/mujoco_course_plan.md) | 课程大纲与整体学习计划 |
| **第一章代码** | [ch1_hello_mujoco.py](file:///d:/code/learning/mujoco-py/ch1_hello_mujoco/ch1_hello_mujoco.py) | 第一章运行代码 |
| **第二章模型** | [ch2_double_pendulum.xml](file:///d:/code/learning/mujoco-py/ch2_double_pendulum/ch2_double_pendulum.xml) | 第二章 XML 模型 |
| **第二章代码** | [ch2_load_pendulum.py](file:///d:/code/learning/mujoco-py/ch2_load_pendulum/ch2_load_pendulum.py) | 第二章加载与状态初始化代码 |
| **第三章代码** | [ch3_free_drop.py](file:///d:/code/learning/mujoco-py/ch3_free_drop/ch3_free_drop.py) | 第三章物理量获取与绘图代码 |
| **第三章原理** | [ch3_python_api_principles.md](file:///d:/code/learning/mujoco-py/ch3_free_drop/ch3_python_api_principles.md) | 第三章原理文档 |
