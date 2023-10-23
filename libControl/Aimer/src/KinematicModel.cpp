#include "../include/KinematicModel.hpp"

namespace wmj
{
    KinematicModel::KinematicModel()
    :   index(0)
    {
        height.clear();
        radius.clear();
    }
    KinematicModel::~KinematicModel()
    {}


    StandardModel::StandardModel()
    :   StandardModel(Eigen::Matrix<double, 10, 1>::Zero())
    {}
    StandardModel::StandardModel(const Eigen::Matrix<double, 10, 1> &X)
    :   KinematicModel()
    {
        height.resize(2);
        radius.resize(2);
        number = 4;

        center = cv::Point2d(X(0, 0), X(2, 0));
        velocity = cv::Point2d(X(1, 0), X(3, 0));
        height[0] = X(4, 0);
        height[1] = X(5, 0);
        radius[0] = X(6, 0);
        radius[1] = X(7, 0);
        phase = X(8, 0);
        palstance = X(9, 0);
    }
    StandardModel::~StandardModel()
    {}

    StandardModel StandardModel::operator=(const StandardModel &status)
    {
        int tmp_index = this->index;
        *this = status;
        this->index = tmp_index;
        return *this;
    }

    Armors StandardModel::getArmors(double predict_time)
    {
        Armors armors(number);
        for (int i = 0; i < number; i++)
        {
            double angle = _std_radian(phase + palstance * predict_time + i * PI / 2);
            cv::Point2d point = center + velocity * predict_time +
                                cv::Point2d(radius[i % 2] * cos(angle), radius[i % 2] * sin(angle));
            armors[i].m_position = cv::Point3d(point.x, point.y, height[i % 2]);
            armors[i].m_yaw_angle = angle;
        }
        return armors;
    }

    Armor StandardModel::getClosestArmor(double predict_time, double switch_threshold)
    {
        /**
         * 我们选择在给定预测时间后在相机系中yaw角度绝对值最小，即面朝我方向最正的一块装甲板作为击打目标
         * 为了防止数据抖动使得目标在两块角度相近的装甲板之间来回跳变，在取最小值时额外加一小段阈值，即需要比原有的最小值减去该阈值更小才认为是新的最小值
         * 但是，这样做会导致目标切换存在一定延迟，所以给求最小值时的时间也额外加一段提前量，即预测其在该一段时间后可能会切换目标，这个时间通过切换阈值和当前的角速度计算得到
         * 另外，为了防止角速度接近0时计算得一个过大的提前量，将上述结果与0.2取较小值
         */
        double switch_advanced_time = std::min(0.2, D2R(switch_threshold) / abs(palstance));
        // std::cout << "switch_advanced_time: " << switch_advanced_time << std::endl;
        Armors armors = getArmors(predict_time + switch_advanced_time);
        cv::Point2d predict_center = center + velocity * predict_time;
        for (int i = 0; i < number; i++)
        {
            if (abs(_std_radian(PI + armors[i].m_yaw_angle - atan2(predict_center.y, predict_center.x))) + D2R(switch_threshold) <
                abs(_std_radian(PI + armors[index].m_yaw_angle - atan2(predict_center.y, predict_center.x))))
            {
                index = i;
            }
        }
        return getArmors(predict_time)[index];
    }

    Armor StandardModel::getFacingArmor(double predict_time)
    {
        Armor armor;
        Armors armors = getArmors(predict_time);
        cv::Point2d predict_center = center + velocity * predict_time;
        for (int i = 0; i < number; i++)
        {
            if (abs(_std_radian(PI + armors[i].m_yaw_angle - atan2(predict_center.y, predict_center.x))) <
                abs(_std_radian(PI + armors[index].m_yaw_angle - atan2(predict_center.y, predict_center.x))))
            {
                index = i;
            }
        }

        // 如果最近的装甲板处于远离状态，则击打下一装甲板
        if (abs(_std_radian(PI + armors[index].m_yaw_angle + (palstance / abs(palstance)) * D2R(-50) - atan2(predict_center.y, predict_center.x))) <
            abs(_std_radian(PI + armors[index].m_yaw_angle - atan2(predict_center.y, predict_center.x))))
        {
            index = (index + 1) % number;
        }
        double angle = _std_radian(atan2(predict_center.y, predict_center.x) - PI);
        armor.m_position = cv::Point3d(predict_center.x + radius[index % 2] * cos(angle), predict_center.y + radius[index % 2] * sin(angle), height[index % 2]);
        armor.m_yaw_angle = angle;
        return armor;
    }

    void StandardModel::print(std::string tag)
    {
        std::string prefix = "[StandardModel] " + tag + " ";
        std::cout << prefix << "center: " << center << " += " << velocity << std::endl;
        std::cout << prefix << "height: " << height[0] << " " << height[1] << std::endl;
        std::cout << prefix << "radius: " << radius[0] << " " << radius[1] << std::endl;
        std::cout << prefix << "angle: " << phase << " += " << palstance << std::endl;
        std::cout << prefix << "index: " << index << std::endl;
    }


    BalanceModel::BalanceModel()
    :   BalanceModel(Eigen::Matrix<double, 7, 1>::Zero())
    {}
    BalanceModel::BalanceModel(const Eigen::Matrix<double, 7, 1> &X)
    :   KinematicModel()
    {
        height.resize(1);
        radius.resize(1);
        number = 2;

        center = cv::Point2d(X(0, 0), X(1, 0));
        velocity = X(2, 0);
        height[0] = X(3, 0);
        radius[0] = X(4, 0);
        phase = X(5, 0);
        palstance = X(6, 0);
    }
    BalanceModel::~BalanceModel()
    {}

    BalanceModel BalanceModel::operator=(const BalanceModel &status)
    {
        int tmp_index = this->index;
        *this = status;
        this->index = tmp_index;
        return *this;
    }

    Armors BalanceModel::getArmors(double predict_time)
    {
        Armors armors(number);
        for (int i = 0; i < number; i++)
        {
            double angle = _std_radian(phase + palstance * predict_time + i * PI);
            cv::Point2d point;
            // if (palstance == 0)
            {
                point.x = center.x + velocity * cos(phase) * predict_time;
                point.y = center.y + velocity * sin(phase) * predict_time;
            }
            // else
            // {
            //     point.x = center.x + velocity / palstance * (sin(palstance * predict_time + phase) - sin(phase));
            //     point.y = center.y - velocity / palstance * (cos(palstance * predict_time + phase) - cos(phase));
            // }
            point += cv::Point2d(radius[0] * cos(angle), radius[0] * sin(angle));
            armors[i].m_position = cv::Point3d(point.x, point.y, height[0]);
            armors[i].m_yaw_angle = angle;
        }
        return armors;
    }

    Armor BalanceModel::getClosestArmor(double predict_time, double switch_threshold)
    {
        // TODO 存在优化空间
        /**
         * 我们选择在给定预测时间后在相机系中yaw角度绝对值最小，即面朝我方向最正的一块装甲板作为击打目标
         * 为了防止数据抖动使得目标在两块角度相近的装甲板之间来回跳变，在取最小值时额外加一小段阈值，即需要比原有的最小值减去该阈值更小才认为是新的最小值
         * 但是，这样做会导致目标切换存在一定延迟，所以给求最小值时的时间也额外加一段提前量，即预测其在该一段时间后可能会切换目标，这个时间通过切换阈值和当前的角速度计算得到
         * 另外，为了防止角速度接近0时计算得一个过大的提前量，将上述结果与0.2取较小值
         */
        double switch_advanced_time = std::min(0.2, D2R(switch_threshold) / abs(palstance));
        // std::cout << "switch_advanced_time: " << switch_advanced_time << std::endl;
        Armors armors = getArmors(predict_time + switch_advanced_time);
        cv::Point2d predict_center = cv::Point2d(center.x + velocity * cos(phase) * predict_time, center.y + velocity * sin(phase) * predict_time);
        for (int i = 0; i < number; i++)
        {
            if (abs(_std_radian(PI + armors[i].m_yaw_angle - atan2(predict_center.y, predict_center.x))) + D2R(switch_threshold) <
                abs(_std_radian(PI + armors[index].m_yaw_angle - atan2(predict_center.y, predict_center.x))))
            {
                index = i;
            }
        }
        return getArmors(predict_time)[index];
    }

    Armor BalanceModel::getFacingArmor(double predict_time)
    {
        Armor armor;
        Armors armors = getArmors(predict_time);
        cv::Point2d predict_center = cv::Point2d(center.x + velocity * cos(phase) * predict_time, center.y + velocity * sin(phase) * predict_time);
        for (int i = 0; i < number; i++)
        {
            if (abs(_std_radian(PI + armors[i].m_yaw_angle - atan2(predict_center.y, predict_center.x))) <
                abs(_std_radian(PI + armors[index].m_yaw_angle - atan2(predict_center.y, predict_center.x))))
            {
                index = i;
            }
        }

        double angle = _std_radian(atan2(predict_center.y, predict_center.x) - PI);
        armor.m_position = cv::Point3d(predict_center.x + radius[0] * cos(angle), predict_center.y + radius[0] * sin(angle), height[0]);
        armor.m_yaw_angle = angle;
        return armor;
    }

    void BalanceModel::print(std::string tag)
    {
        std::string prefix = "[BalanceModel] " + tag + " ";
        std::cout << prefix << "center: " << center << " += " << velocity << std::endl;
        std::cout << prefix << "height: " << height[0] << std::endl;
        std::cout << prefix << "radius: " << radius[0] << std::endl;
        std::cout << prefix << "angle: " << phase << " += " << palstance << std::endl;
        std::cout << prefix << "index: " << index << std::endl;
    }

    OutpostModel::OutpostModel()
    :   OutpostModel(Eigen::Matrix<double, 5, 1>::Zero())
    {}
    OutpostModel::OutpostModel(const Eigen::Matrix<double, 5, 1> &X)
    :   KinematicModel()
    {
        height.resize(1);
        radius.resize(1);
        number = 3;

        center = cv::Point2d(X(0, 0), X(1, 0));
        height[0] = X(2, 0);
        radius[0] = 0.25685;
        phase = X(3, 0);
        palstance = X(4, 0);
    }
    OutpostModel::~OutpostModel()
    {}

    OutpostModel OutpostModel::operator=(const OutpostModel &status)
    {
        int tmp_index = this->index;
        *this = status;
        this->index = tmp_index;
        return *this;
    }

    Armors OutpostModel::getArmors(double predict_time)
    {
        Armors armors(number);
        for (int i = 0; i < number; i++)
        {
            double angle = _std_radian(phase + palstance * predict_time + 2 * i * PI / 3.0);
            cv::Point2d point = center + cv::Point2d(radius[0] * cos(angle), radius[0] * sin(angle));
            armors[i].m_position = cv::Point3d(point.x, point.y, height[0]);
            armors[i].m_yaw_angle = angle;
        }
        return armors;
    }

    Armor OutpostModel::getClosestArmor(double predict_time, double switch_threshold)
    {
        /**
         * 我们选择在给定预测时间后在相机系中yaw角度绝对值最小，即面朝我方向最正的一块装甲板作为击打目标
         * 为了防止数据抖动使得目标在两块角度相近的装甲板之间来回跳变，在取最小值时额外加一小段阈值，即需要比原有的最小值减去该阈值更小才认为是新的最小值
         * 但是，这样做会导致目标切换存在一定延迟，所以给求最小值时的时间也额外加一段提前量，即预测其在该一段时间后可能会切换目标，这个时间通过切换阈值和当前的角速度计算得到
         * 另外，为了防止角速度接近0时计算得一个过大的提前量，将上述结果与0.2取较小值
         */
        double switch_advanced_time = std::min(0.2, D2R(switch_threshold) / abs(palstance));
        // std::cout << "switch_advanced_time: " << switch_advanced_time << std::endl;
        Armors armors = getArmors(predict_time + switch_advanced_time);
        for (int i = 0; i < number; i++)
        {
            if (abs(_std_radian(PI + armors[i].m_yaw_angle - atan2(center.y, center.x))) + D2R(switch_threshold) <
                abs(_std_radian(PI + armors[index].m_yaw_angle - atan2(center.y, center.x))))
            {
                index = i;
            }
        }
        return getArmors(predict_time)[index];
    }

    Armor OutpostModel::getFacingArmor(double predict_time)
    {
        Armor armor;
        Armors armors = getArmors(predict_time);
        for (int i = 0; i < number; i++)
        {
            if (abs(_std_radian(PI + armors[i].m_yaw_angle - atan2(center.y, center.x))) <
                abs(_std_radian(PI + armors[index].m_yaw_angle - atan2(center.y, center.x))))
            {
                index = i;
            }
        }

        double angle = _std_radian(atan2(center.y, center.x) - PI);
        armor.m_position = cv::Point3d(center.x + radius[0] * cos(angle), center.y + radius[0] * sin(angle), height[0]);
        armor.m_yaw_angle = angle;
        return armor;
    }

    void OutpostModel::print(std::string tag)
    {
        std::string prefix = "[OutpostModel] " + tag + " ";
        std::cout << prefix << "center: " << center << std::endl;
        std::cout << prefix << "height: " << height[0] << std::endl;
        std::cout << prefix << "radius: " << radius[0] << std::endl;
        std::cout << prefix << "angle: " << phase << " += " << palstance << std::endl;
        std::cout << prefix << "index: " << index << std::endl;
    }

}   // wmj
