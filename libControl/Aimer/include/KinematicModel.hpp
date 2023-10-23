#pragma once

#include "../../../libBase/include/common.h"

// 将弧度约束在[-pi, pi]范围内
#ifndef _std_radian
#define _std_radian(angle) ((angle) + round((0 - (angle)) / (2 * PI)) * (2 * PI))
#endif


namespace wmj
{
    /**
     * @brief 运动学状态
     */
    class KinematicModel
    {
    public:
        KinematicModel();
        ~KinematicModel();

        virtual Armors getArmors(double predict_time = 0) = 0;
        virtual Armor getClosestArmor(double predict_time, double switch_threshold) = 0;
        virtual Armor getFacingArmor(double predict_time) = 0;

        virtual void print(std::string tag) = 0;

    public:
        cv::Point2d center;             // 旋转中心位置
        std::vector<double> height;     // 装甲高度。0对应索引0、2，1对应索引1、3
        std::vector<double> radius;     // 旋转半径。0对应索引0、2，1对应索引1、3
        double phase;                   // 角度（并非定值）
        double palstance;               // 角速度

        int number;                     // 装甲板数量
        int index;                      // 当前目标索引

    public:
        enum Type
        {
            UNKNOWN = -1,
            STANDARD = 0,
            BALANCE = 1,
            OUTPOST = 2
        };
    };

    class StandardModel : public KinematicModel
    {
    public:
        StandardModel();
        StandardModel(const Eigen::Matrix<double, 10, 1> &X);
        ~StandardModel();

        /**
         * @brief 为了保留index，赋值时不继承index
         */
        StandardModel operator=(const StandardModel &status);

        /**
         * @brief 获取该运动状态下所有装甲板
         * @param predict_time 预测时间
         */
        Armors getArmors(double predict_time = 0);

        /**
         * @brief 获取目标装甲板
         * @param predict_time 预测时间
         * @param switch_threshold 更新装甲板切换的最小距离差
         */
        Armor getClosestArmor(double predict_time, double switch_threshold);

        /**
         * @brief 获取正对的装甲板
         */
        Armor getFacingArmor(double predict_time);

        /**
         * @brief 打印信息
         */
        void print(std::string tag);

    public:
        cv::Point2d velocity;           // 旋转中心速度
    };

    class BalanceModel : public KinematicModel
    {
    public:
        BalanceModel();
        BalanceModel(const Eigen::Matrix<double, 7, 1> &X);
        ~BalanceModel();

        /**
         * @brief 为了保留index，赋值时不继承index
         */
        BalanceModel operator=(const BalanceModel &status);

        /**
         * @brief 获取该运动状态下所有装甲板
         * @param predict_time 预测时间
         */
        Armors getArmors(double predict_time = 0);

        /**
         * @brief 获取目标装甲板
         * @param predict_time 预测时间
         * @param switch_threshold 更新装甲板切换的最小距离差
         */
        Armor getClosestArmor(double predict_time, double switch_threshold);

        /**
         * @brief 获取正对的装甲板
         */
        Armor getFacingArmor(double predict_time);

        /**
         * @brief 打印信息
         */
        void print(std::string tag);

    public:
        double velocity;                // 旋转中心速度
    };

    class OutpostModel : public KinematicModel
    {
    public:
        OutpostModel();
        OutpostModel(const Eigen::Matrix<double, 5, 1> &X);
        ~OutpostModel();

        /**
         * @brief 为了保留index，赋值时不继承index
         */
        OutpostModel operator=(const OutpostModel &status);

        /**
         * @brief 获取该运动状态下所有装甲板
         * @param predict_time 预测时间
         */
        Armors getArmors(double predict_time = 0);

        /**
         * @brief 获取目标装甲板
         * @param predict_time 预测时间
         * @param switch_threshold 更新装甲板切换的最小距离差
         */
        Armor getClosestArmor(double predict_time, double switch_threshold);

        /**
         * @brief 获取正对的装甲板
         */
        Armor getFacingArmor(double predict_time);

        /**
         * @brief 打印信息
         */
        void print(std::string tag);
    };

}   // wmj
