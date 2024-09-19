# README

西北工业大学 WMJ 战队 2023 赛季自瞄预测器开源代码，目前核心部分仍在投入比赛使用。

## 仓库结构

```
.
├── libBase                 // 基础单元库，包含时间、键盘事件、彩色字符相关等常用函数和Armor、GimbalPose等基本数据类型定义
├── libControl              // 控制单元库，包含各种控制解算器
│   ├── Pose                // 云台位姿解算器类
│   └── TopAimer            // 反陀螺预测器类
├── log                     // 用于存放日志文件
├── test                    // 用于存放测试代码
├── video                   // 用于存放录制视频
├── CMakeLists.txt
├── README.md
└── setup                   // 初始化仓库脚本
```

## 运行环境

| Recommended OS | Dependency Library        | Compiler                     |
| -------------- | ------------------------- | ---------------------------- |
| Ubuntu 18.04+  | Eigen 3+<br />OpenCV 3.4+ | g++ or clang++<br />CMake 3+ |

## 运行方式

直接运行 `setup` 脚本或者手动执行以下步骤

```shell
mkdir build && cd build
cmake .. && make -j
```

编译完成后根据需要执行测试程序，测试程序位于 `bin/`

## 项目展示

### 效果展示

- 静态目标

![static_target](https://github.com/SnocrashWang/WMJAimer/blob/master/libControl/Aimer/image/static_target.gif)

- 动态目标

![dynamic_target](https://github.com/SnocrashWang/WMJAimer/blob/master/libControl/Aimer/image/dynamic_target.gif)

### 开发记录

[开发记录](https://www.bilibili.com/video/BV1ha4y1M7Jy/?spm_id_from=333.337.search-card.all.click&vd_source=9d4b3ad031f43830dad4944d85c1384f)

## 备注

由于本项目节选自 WMJ 战队步兵视觉代码仓库，部分内容无法完整编译，仅作展示
