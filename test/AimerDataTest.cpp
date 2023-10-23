/*************************************************************
 * 
 * 反陀螺预测器数据测试
 * @Author 王云飞，王铭远，黄永乐
 * 
 * 本测试程序编写始于2022年夏复活赛备赛期间，主要用于反陀螺预测器相关的数据测试和验证，功能包括：
 * 1. 在仿真的综合运动中验证EKF状态观测器的性能
 * 2. 测试不同运动模型的观测效果
 * 3. 使用录制数据运行算法并进行调参
 * 4. 验证MPC控制器
 * 
 * 本测试程序将产生两个窗口，其一为平面综合运动仿真，其二为MPC解算数据图。
 *  在仿真图中
 *  图像中心灰色点为观测点，黑色圆点表示目标旋转中心
 *  蓝色点表示含噪的四个装甲板坐标，与观测中心有黑色连线者表示被输入到预测器中的装甲板；绿色点表示当前状态观测量；红色点表示预测坐标，被紫色圆圈圈起来的表示当前击打目标
 *  键盘WASD可移动目标位置，JK可调整旋转速度，GH分别重置水平速度和角速度为0
 * 
 *  在MPC数据图中
 *  蓝色与浅蓝色分别表示云台当前的俯仰角和偏航角
 *  红色与橙色分别表示根据解算目标位置云台应有的俯仰角和偏航角
 *  绿色和紫色分别表示当前MPC输出的云台俯仰角和偏航角的速度控制量
 * 
 *  log/ 中提供了部分我们实际录制的观测数据，可以用于验证算法；录制数据的格式可以参考测试代码AimerTest.cpp中的录制数据部分
 * 
 */

#include "../libControl/Aimer/include/Aimer.hpp"

// 仿真平衡模型
// #define _BALANCE
// 仿真前哨站模型
// #define _OUTPOST
// 默认使用标准模型

// 使用录制数据进行仿真
// #define USE_DATA

// 开启仿真录制
// #define REC_VIDEO

// 观察点坐标
cv::Point2f observation_point(0, 0);
cv::RNG rng(unsigned(time(NULL)));
// 噪声大小
double sigma = 0.01;
// 每帧时间
float dt = 1e-2;
// 射速
float bullet_speed = 13.5;
#ifdef _BALANCE
cv::Point2f center(2, 2);
float z = 0.15;
float r = 0.15;
float velocity = 0;
float palstance = 1;
float scaling = 100;
wmj::KinematicModel::Type type = wmj::KinematicModel::Type::BALANCE;
#else
#ifdef _OUTPOST
cv::Point2f center(5 + rng.gaussian(1), 5 + rng.gaussian(1));
float z = 0.65;
float r = 0.25685;
float palstance = 0.4;
float scaling = 50;
wmj::KinematicModel::Type type = wmj::KinematicModel::Type::OUTPOST;
#else
// 中心点
cv::Point2f center(2, 2);
// 装甲板高度
float z[2] = {0.15, 0.1};
// 装甲板旋转半径
float r[2] = {0.2, 0.3};
// 水平速度
cv::Point2f velocity(0, 0);
// 角速度
float palstance = 1;
// 放大比例
float scaling = 100;
// 运动模型
wmj::KinematicModel::Type type = wmj::KinematicModel::Type::STANDARD;
#endif
#endif
// 起始角度
double angle = 0;
// 键盘输入
char c;
// 仿真画面
cv::Mat background;
// MPC解算数据
cv::Mat figure_mpc = cv::Mat(cv::Size(1024, 512), CV_8UC3, cv::Scalar(255, 255, 255));
// MPC解算计数（超出图表范围后自动重置）
int cnt = 0;
// 原点（画面中心点）
cv::Point2f base_point = cv::Point2f(512, 512);
// 动画帧率
wmj::Rate rate(100);
// 上一帧速度，用于MPC绘图
wmj::GimbalPose last_speed(0, 0);

// 获取装甲板序列
wmj::Armors getArmors()
{
    wmj::Armors armors;
#ifdef _BALANCE
    armors.resize(2);
    if (palstance == 0)
    {
        center.x += velocity * cos(angle) * dt;
        center.y += velocity * sin(angle) * dt;
    }
    else
    {
        center.x += velocity / palstance * (sin(palstance * dt + angle) - sin(angle));
        center.y += velocity / palstance * (-cos(palstance * dt + angle) + cos(angle));
    }
    angle += PI * palstance * dt;

    for (int i = 0; i < armors.size(); i++)
    {
        armors[i].m_position = cv::Point3f(
            center.x + r * cos(angle + i * PI) + rng.gaussian(sigma),
            center.y + r * sin(angle + i * PI) + rng.gaussian(sigma),
            z + rng.gaussian(sigma));
        armors[i].m_yaw_angle = _std_radian(angle + rng.gaussian(sigma * 10) + i * PI);
    }
#else
#ifdef _OUTPOST
    armors.resize(3);
    center = center;
    angle += 2 * PI * dt * palstance;

    for (int i = 0; i < armors.size(); i++)
    {
        armors[i].m_position = cv::Point3f(
            center.x + r * cos(angle + 2 * i * PI / 3) + rng.gaussian(sigma),
            center.y + r * sin(angle + 2 * i * PI / 3) + rng.gaussian(sigma),
            z + rng.gaussian(sigma));
        armors[i].m_yaw_angle = _std_radian(angle + rng.gaussian(sigma * 10) + 2 * i * PI / 3);
    }
#else
    armors.resize(4);
    center = center + velocity;
    angle += PI * dt * palstance;

    for (int i = 0; i < armors.size(); i++)
    {
        armors[i].m_position = cv::Point3f(
            center.x + r[i % 2] * cos(angle + 0.5 * i * PI) + rng.gaussian(sigma),
            center.y + r[i % 2] * sin(angle + 0.5 * i * PI) + rng.gaussian(sigma),
            z[i % 2] + rng.gaussian(sigma));
        armors[i].m_yaw_angle = _std_radian(angle + rng.gaussian(sigma * 10) + 0.5 * i * PI);
    }
#endif
#endif
    // 目标旋转中心
    cv::circle(background, base_point + center * scaling, 5, cv::Scalar(0, 0, 0), -1, 8);
    // 观察点
    cv::circle(background, base_point + observation_point * scaling, 5, cv::Scalar(127, 127, 127), -1, 8);
    for (int i = 0; i < armors.size(); i++)
    {
        // 蓝色为观测输入
        // 四块装甲板坐标（含噪）
        cv::circle(background, base_point + cv::Point2f(armors[i].m_position.x, armors[i].m_position.y) * scaling, 3,
                    cv::Scalar(255, 0 , 0), -1, 8);
        // 装甲板朝向角（含噪）
        cv::line(background, base_point + cv::Point2f(armors[i].m_position.x, armors[i].m_position.y) * scaling,
                    base_point + cv::Point2f(armors[i].m_position.x, armors[i].m_position.y) * scaling + cv::Point2f(10 * cos(armors[i].m_yaw_angle), 10 * sin(armors[i].m_yaw_angle)),
                    cv::Scalar(255, 0 , 0), 2, 8);
    }
    // 将观测数据随机删除至距离自身最近的0~2个，可测试其高压下的鲁棒性
    // std::sort(armors.begin(), armors.end(),
    //           [](wmj::Armor &a, wmj::Armor &b)
    //           {
    //               if (std::pow(a.m_position.x, 2) + std::pow(a.m_position.y, 2) > std::pow(b.m_position.x, 2) + std::pow(b.m_position.y, 2))
    //                   return false;
    //               else
    //                   return true;
    //           });
    // armors.resize(rng.uniform(0, 3));
    // 确保作为观测输入的装甲板角度不会太偏
    wmj::Armors select_armors;
    for (int i = 0; i < armors.size(); i++)
    {
        if (abs(_std_radian(PI + armors[i].m_yaw_angle - atan2(center.y, center.x))) < D2R(50))
        {
            select_armors.emplace_back(armors[i]);
            std::cout << "Input armor: " << armors[i].m_position << " " << armors[i].m_yaw_angle << std::endl;
            cv::line(background, base_point + cv::Point2f(armors[i].m_position.x, armors[i].m_position.y) * scaling, base_point + observation_point * scaling, cv::Scalar(127, 127, 127), 1);
        }
    }

    return select_armors;
}


int main()
{
    // 预测器
    wmj::Aimer aimer;
    aimer.setModelType(type);
    // 角度解算器
    wmj::AngleSolver angle;
    // 运动状态
    std::shared_ptr<wmj::KinematicModel> status;
    // 当前云台位姿
    wmj::GimbalPose cur_pose;
    // 录制数据路径
    std::ifstream ifs("../log/Aimer_Record50.txt");
#ifdef REC_VIDEO
    // 仿真效果视频导出路径
    cv::VideoWriter recorder("../video/ekf" + std::to_string((int)wmj::now() % 1000) + ".avi", CV_FOURCC('M', 'J', 'P', 'G'), 60, cv::Size(1024, 1024), true);
#endif
    // while (true)
    while (!ifs.eof())
    {
        background = cv::Mat(cv::Size(1024, 1024), CV_8UC3, cv::Scalar(255, 255, 255));
#ifndef USE_DATA
        wmj::Armors armors = getArmors();
#else
        wmj::Armors armors;
        wmj::GimbalPose cur_pose;
        double bullet_speed;
        ifs >> cur_pose.pitch >> cur_pose.yaw >> bullet_speed;
        for(int i = 0; i < 5; i++)
        {
            double x, y, z;
            double yaw;
            std::string yaws;
            ifs >> x >> y >> z >> yaws;
            if(yaws=="nan")
                yaw = NAN;
            else
                yaw = atof(yaws.c_str());
            wmj::Armor armor;
            armor.m_position = angle.cam2abs(cv::Point3f(x, y, z), cur_pose);
            armor.m_yaw_angle = _std_radian(yaw + cur_pose.yaw + PI);
            if (x != 0
                && abs(yaw) < D2R(60)
                )
            {
                armors.emplace_back(armor);
                cv::circle(background, base_point + cv::Point2f(armor.m_position.x, armor.m_position.y) * scaling, 3,
                            cv::Scalar(255,0 , 0), -1, 8);
                cv::line(background, base_point + cv::Point2f(armor.m_position.x, armor.m_position.y) * scaling,
                            base_point + cv::Point2f(armor.m_position.x, armor.m_position.y) * scaling + cv::Point2f(10 * cos(armor.m_yaw_angle), 10 * sin(armor.m_yaw_angle)),
                            cv::Scalar(255,0 , 0), 2, 8);
            }
        }
#endif
        // 若第一帧数据为空则跳过
        if (!status && armors.empty())
            continue;

        // 处理异常数据
        aimer.errorHandling(armors);
        // 预测器解算
        status = aimer.resolve(armors);
        // 当前状态量
        wmj::Armors filter_armors = status->getArmors();
        double hit_time = getDistance(status->getClosestArmor(0, 0).m_position) / bullet_speed + 0;
        // 预测坐标
        wmj::Armors predict_armors = status->getArmors(hit_time);
        // 目标坐标
        wmj::Armor target_armor = status->getClosestArmor(hit_time, 0.05);
            // std::cout << "index: " << status->index << std::endl;
            std::cout << "precise_position: " << target_armor.m_position << std::endl;
        wmj::GimbalPose precise_pose = angle.getAngle(angle.abs2cam(target_armor.m_position, cur_pose), cur_pose, bullet_speed);
            std::cout << "precise_pose: " << precise_pose << std::endl;
            std::cout << "current_pose: " << cur_pose << std::endl;
        // MPC速度控制量
        wmj::GimbalPose target_speed = aimer.m_MPC->getGimbalSpeed(status, cur_pose, bullet_speed);
            // std::cout << "ctrl_speed: " << target_speed << std::endl;
            // if (abs(target_speed.yaw) > 5)
            //     std::cout << _warning("speed step detected!!!") << std::endl;
        // 更新当前位姿
        cur_pose = cur_pose + 0.01 * target_speed;
        cur_pose = cur_pose + wmj::GimbalPose(rng.gaussian(0.01), rng.gaussian(0.01));
        cur_pose.pitch = _std_radian(cur_pose.pitch);
        cur_pose.yaw = _std_radian(cur_pose.yaw);
        precise_pose.pitch = _std_radian(precise_pose.pitch);
        precise_pose.yaw = _std_radian(precise_pose.yaw);

        // 基线
        cv::line(figure_mpc, cv::Point(0, 256), cv::Point(1024, 256), cv::Scalar(0, 0, 0), 1, 8);
        // 当前云台位姿
        cv::circle(figure_mpc, cv::Point2d(cnt, 256 - 50 * cur_pose.pitch), 1, cv::Scalar(0, 0, 255), -1, 8);
        cv::circle(figure_mpc, cv::Point2d(cnt, 256 - 50 * cur_pose.yaw), 1, cv::Scalar(0, 127, 255), -1, 8);
        // 精确云台位姿
        cv::circle(figure_mpc, cv::Point2d(cnt, 256 - 50 * precise_pose.pitch), 1, cv::Scalar(255, 0, 0), -1, 8);
        cv::circle(figure_mpc, cv::Point2d(cnt, 256 - 50 * precise_pose.yaw), 1, cv::Scalar(255, 127, 0), -1, 8);
        // 速度控制量
        cv::line(figure_mpc, cv::Point2d(cnt, 256 - 20 * last_speed.pitch), cv::Point2d(cnt, 256 - 20 * target_speed.pitch), cv::Scalar(0, 255, 0), 1, 8);
        cv::line(figure_mpc, cv::Point2d(cnt, 256 - 20 * last_speed.yaw), cv::Point2d(cnt, 256 - 20 * target_speed.yaw), cv::Scalar(255, 0, 255), 1, 8);

        last_speed = target_speed;

        for (int i = 0; i < filter_armors.size(); i++)
        {
            // 当前状态估计量
            cv::circle(background, base_point + cv::Point2f(filter_armors[i].m_position.x, filter_armors[i].m_position.y) * scaling, 3,
                    cv::Scalar(0, 255, 0), -1, 8);
            // 预测装甲板坐标
            cv::circle(background, base_point + cv::Point2f(predict_armors[i].m_position.x, predict_armors[i].m_position.y) * scaling, 2,
                    cv::Scalar(0, 0, 255), -1, 8);
            // 预测装甲板高度
            // cv::circle(background, base_point + cv::Point2f(predict_armors[i].m_position.x, predict_armors[i].m_position.y) * scaling, abs(predict_armors[i].m_position.z) * 50,
            //         cv::Scalar(0, 127, 255), 2, 8);
        }
        double relative_angle = abs(_std_radian(PI + target_armor.m_yaw_angle - atan2(target_armor.m_position.y, target_armor.m_position.x)));
        // 目标装甲板
        if (relative_angle < D2R(40))
            cv::circle(background, base_point + cv::Point2f(target_armor.m_position.x, target_armor.m_position.y) * scaling, 10,
                    cv::Scalar(255, 0, 255), 2, 8);

        if (c == 'q')
            break;
        switch (c)
        {
#ifdef _BALANCE
        case 'w':
            velocity += 1;
            break;
        case 's':
            velocity -= 1;
            break;
        case 'g':
            velocity = 0;
            break;
        case 'a':
            palstance -= 1;
            break;
        case 'd':
            palstance += 1;
            break;
        case 'h':
            palstance = 0;
            break;
#else
#ifdef _OUTPOST
        case 'a':
            palstance = 0.4;
            break;
        case 'd':
            palstance = 0.2;
            break;
        case 'h':
            palstance = 0;
            break;
#else
        case 'w':
            velocity.y -= dt;
            break;
        case 's':
            velocity.y += dt;
            break;
        case 'a':
            velocity.x -= dt;
            break;
        case 'd':
            velocity.x += dt;
            break;
        case 'g':
            velocity = cv::Point2f(0, 0);
            break;
        case 'j':
            palstance -= 1;
            break;
        case 'k':
            palstance += 1;
            break;
        case 'h':
            palstance = 0;
            break;
#endif
#endif

        default:
            break;
        }

        cv::imshow("background", background);
        cv::imshow("pose", figure_mpc);

#ifdef REC_VIDEO
        recorder << background;
#endif
        // 若数据量超出范围则重置图表
        cnt++;
        if (cnt > 1024)
        {
            cnt = 0;
            figure_mpc = cv::Mat(cv::Size(1024, 512), CV_8UC3, cv::Scalar(255, 255, 255));
        }
        std::cout << "------" << std::endl;

#ifdef USE_DATA
        c = cv::waitKey(0);
#else
        c = cv::waitKey(1);
        // 控制运行帧率
        rate.sleep();
#endif
    }

#ifdef REC_VIDEO
    recorder.release();
#endif
    return 0;
}