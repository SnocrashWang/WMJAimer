# 反陀螺预测器

## 代码说明

### 部分引用数据类型

```c++
/**
 * @brief 装甲板
 */
struct Armor
{
	int m_id;                // ID
	double m_yaw_angle;      // yaw轴旋转角
	cv::Point3f m_position;  // 坐标
};

/**
 * @brief 云台位姿
 */
struct GimbalPose
{
    float pitch;             // 俯仰角
    float yaw;               // 偏航角
    double timestamp;        // 时间戳
};
```

### Aimer

反陀螺主模块，主要包括自瞄状态判断、数据预处理、目标匹配、重投影 ROI 计算和发射判断部分。

#### 核心接口

```c++
/**
 * @brief 模块主接口。
 * @param armors 相机坐标系装甲板序列
 * @param cur_pose 当前位姿
 * @param bullet_speed 当前射速
 * @return 目标位姿
 */
GimbalPose Aimer::getTargetPose(const Armors &armors, const GimbalPose &cur_pose, double bullet_speed);
```

#### 控制接口

```c++
/**
 * @brief 完全重置
 */
void Aimer::reset();

/**
 * @return 是否允许控制
 */
bool Aimer::isReady();

/**
 * @return 是否允许击发
 */
bool Aimer::shootable();
```

#### 其他接口

```c++
/**
 * @brief 数据解算核心过程，同时也是提供给数据模拟器的主接口
 * @param armors 装甲板序列
 * @return 运动学状态
 */
std::shared_ptr<KinematicModel> Aimer::resolve(const Armors &armors);

/**
 * @brief 抛弃装甲板序列中数据异常者
 * @param armors 用于处理的装甲板序列
 */
void Aimer::errorHandling(Armors &armors);

/** 
 * @brief 强制指定运动学模型并实例化，慎用
 */
void Aimer::setModelType(KinematicModel::Type type);
```

### KinematicModel

该类用于存放描述目标运动学状态的详细信息，包括旋转中心、平面速度、旋转速度、装甲板半径等，以及返回所需装甲板坐标的函数接口。

注：由于抽象类不允许重载运算符的虚函数，故派生类中须额外声明并定义如下重载运算符

```c++
/**
 * @brief 为了保留index，赋值时不继承index
 */
MyModel MyModel::operator=(const MyModel &status)
{
    int tmp_index = this->index;
    *this = status;
    this->index = tmp_index;
    return *this;
}
```

### ModelObserver

该类用于通过 EKF 计算状态预测和更新，需与 KinematicModel 类一一对应。