# 位姿解算器

## 主要接口

```c++
wmj::GimbalPose wmj::AngleSolver::getAngle(wmj::Armor armor, wmj::GimbalPose cur_pose, double bullet_speed)
```

参数：

- armor: 目标装甲板
- cur_pose: 当前云台位姿位姿
- bullet_speed: 当前射速

返回：

​	目标云台位姿

```c++
cv::Point3f wmj::AngleSolver::cam2abs(cv::Point3f position, wmj::GimbalPose cur_pose)
```

参数：

- position: 相机系三维坐标
- cur_pose: 当前云台位姿

返回：

​	地面系三维坐标

```c++
cv::Point3f wmj::AngleSolver::abs2cam(cv::Point3f position, wmj::GimbalPose cur_pose)
```

参数：

- position: 地面系三维坐标
- cur_pose: 当前云台位姿

返回：

​	相机系三维坐标

## 工作原理

1. 将相机系相对坐标转化为枪口系相对坐标（相差一个平移矩阵）
2. 根据枪口系相对坐标解算相对欧拉角
3. 将当前位姿加上相对欧拉角得到目标位姿
4. 根据当前射速计算弹道补偿，修正pitch值

## 调参方法

1. 从机械图纸得到相机系到枪口系的平移矩阵
2. 先关闭弹道补偿，根据激光红点位置微调pitch_off和yaw_off，在不同距离都能将红点对准装甲板中心
3. 打开弹道补偿，在不同距离不同高度不同射速下测试子弹命中位置。记得检查测速射速和公式中的射速是否吻合