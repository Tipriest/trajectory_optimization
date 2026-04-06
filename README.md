# README.md

<div align="center" style="margin: 20px 0;">
  <img src="assets/intro.gif"
       alt="intro image"
       title="intro image for vln gazebo simulator"
       width="800"
       style="max-width: 100%; height: auto; border-radius: 8px; box-shadow: 0 4px 12px rgba(0,0,0,0.15);"
       loading="lazy"/>
</div>

## 一. 项目作用
根据库map_generator生成的grid_map，进行全局路径搜索，根据全局路径搜索的结果，生成凸包作为飞行走廊，根据飞行走廊的结果，作为MPC的约束，优化后面的MPC的轨迹


## 二. 环境安装
> 此设置已在 **Ubuntu 20.04** 和 **ros1 noetic** 上通过测试。

### 2.1 克隆带有子模块的仓库
```bash
mkdir -p traj_optimization_ws/src && cd traj_optimization_ws
git clone git@github.com:Tipriest/trajectory_optimization.git ./src/trajectory_optimization
cd ./src/trajectory_optimization && git submodule update --init --recursive
cd .. && git clone git@github.com:Tipriest/map_generator.git

sudo apt install ros-noetic-grid-map
```

### 2.2 编译
```bash
catkin build
```


## 三. 仿真环境使用
```bash
# 打开仿真地图
roslaunch map_generator map_generator.launch
roslaunch trajectory_optimization optimize_visual.launch

```

## 四. 起点/终点选择方式

支持两种模式，通过参数 `start_mode` 配置：

- `two_click`：在 RViz 里连续点两次，第一次为起点，第二次为终点
- `odom_plus_click`：起点来自 `odom_topic`（默认 `/gazebo_odom`），RViz 只点一次作为终点

对应参数文件：

- [trajectory_optimization/config/trajectory_optimization.yaml](trajectory_optimization/config/trajectory_optimization.yaml)

常用参数：

- `start_mode`：`two_click` 或 `odom_plus_click`
- `goal_topic`：默认 `/move_base_simple/goal`
- `odom_topic`：默认 `/gazebo_odom`
- `marker_frame_id`：起点/终点 marker 的坐标系
- `start_marker_*` / `goal_marker_*`：marker 颜色 (RGBA) 与大小

示例：

```yaml
start_mode: "odom_plus_click"
goal_topic: "/move_base_simple/goal"
odom_topic: "/gazebo_odom"
marker_frame_id: "world"
start_marker_r: 0.0
start_marker_g: 1.0
start_marker_b: 0.0
start_marker_a: 1.0
start_marker_scale: 0.5
goal_marker_r: 1.0
goal_marker_g: 0.0
goal_marker_b: 0.8
goal_marker_a: 1.0
goal_marker_scale: 0.5
```
