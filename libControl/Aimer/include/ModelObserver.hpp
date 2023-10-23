#pragma once

#include "../include/KinematicModel.hpp"

namespace wmj
{
    class ModelObserver
    {
    public:
        ModelObserver();
        ~ModelObserver();

        virtual std::pair<Eigen::MatrixXd, Eigen::MatrixXd> predict(const Armors &armors) = 0;
        virtual std::shared_ptr<KinematicModel> update(const Armors &armors, const Eigen::MatrixXd &X_, const Eigen::MatrixXd &P_, std::map<int, int> &match) = 0;

        virtual void reset() = 0;
        virtual bool stable() = 0;

    protected:
        virtual void setParam(const std::string &file_path) = 0;

        virtual Eigen::MatrixXd getPredictiveMeasurement(const Eigen::MatrixXd &X, int i) = 0;
        virtual Eigen::MatrixXd getMeasurementPD(const Eigen::MatrixXd &X, int i) = 0;
        virtual Eigen::MatrixXd getMeasureNoisePD(const Eigen::MatrixXd &X, int i) = 0;

    protected:
        bool m_debug;
        bool m_init;

        // 参数
        double m_dt;                                // 单位时间
        double m_init_radius;                       // 初始半径
        double m_gain;                              // 测距噪声关于角度的增益倍数

        double m_process_noise[4];                  // 状态转移噪声系数
        double m_measure_noise[3];                  // 观测噪声系数

        Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> m_X;          // 状态
        Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> m_X_update;   // 状态修正
        Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> m_P;          // 状态协方差
        Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> m_F;          // 状态转移矩阵
        Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> m_Q;          // 状态转移噪声
        Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> m_R;          // 观测噪声
    };

    class StandardObserver : public ModelObserver
    {
    public:
        StandardObserver();
        ~StandardObserver();

        /**
         * @brief 先验预测
         * @param armors 绝对坐标系装甲板序列
         * @return std::pair<Eigen::Matrix<double, 10, 1>, Eigen::Matrix<double, 10, 10>>(X_, P_)
         */
        std::pair<Eigen::MatrixXd, Eigen::MatrixXd> predict(const Armors &armors);

        /**
         * @brief 匹配装甲板序列和标准装甲板
         * @param armors 绝对坐标系装甲板序列
         * @param X_ Eigen::Matrix<double, 10, 1> 先验状态
         * @param P_ Eigen::Matrix<double, 10, 10> 先验状态协方差
         * @param match 装甲板关联
         * @return 运动状态
         */
        std::shared_ptr<KinematicModel> update(const Armors &armors, const Eigen::MatrixXd &X_, const Eigen::MatrixXd &P_, std::map<int, int> &match);

        /**
         * @brief 重置
         */
        void reset();

        /**
         * @return 模型是否稳定
         */
        bool stable();

    private:
        /**
         * @brief 读取参数配置文件
         * @param file_path 配置文件路径
         */
        void setParam(const std::string &file_path);

        /**
         * @brief 获取先验观测量
         * @param X Eigen::Matrix<double, 10, 1> 先验状态
         * @param i 标准装甲板索引
         * @return Eigen::Matrix<double, 4, 1> 先验观测矩阵
         */
        Eigen::MatrixXd getPredictiveMeasurement(const Eigen::MatrixXd &X, int i);

        /**
         * @brief 获取观测方程偏导矩阵
         * @param X Eigen::Matrix<double, 10, 1> 先验状态
         * @param i 标准装甲板索引
         * @return Eigen::Matrix<double, 4, 10> 观测偏导矩阵
         */
        Eigen::MatrixXd getMeasurementPD(const Eigen::MatrixXd &X, int i);

        /**
         * @brief 获取观测噪声偏导矩阵
         * @param X Eigen::Matrix<double, 10, 1> 先验状态
         * @param i 标准装甲板索引
         * @return Eigen::Matrix<double, 4, 4> 观测偏导矩阵
         */
        Eigen::MatrixXd getMeasureNoisePD(const Eigen::MatrixXd &X, int i);

    private:
        Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> m_RR;         // 增广观测噪声
    };

    class BalanceObserver : public ModelObserver
    {
    public:
        BalanceObserver();
        ~BalanceObserver();

        /**
         * @brief 先验预测
         * @param armors 绝对坐标系装甲板序列
         * @return std::pair<Eigen::Matrix<double, 7, 1>, Eigen::Matrix<double, 7, 7>>(X_, P_)
         */
        std::pair<Eigen::MatrixXd, Eigen::MatrixXd> predict(const Armors &armors);

        /**
         * @brief 匹配装甲板序列和标准装甲板
         * @param armors 绝对坐标系装甲板序列
         * @param X_ Eigen::Matrix<double, 7, 1> 先验状态
         * @param P_ Eigen::Matrix<double, 7, 7> 先验状态协方差
         * @param match 装甲板关联
         * @return 运动状态
         */
        std::shared_ptr<KinematicModel> update(const Armors &armors, const Eigen::MatrixXd &X_, const Eigen::MatrixXd &P_, std::map<int, int> &match);

        /**
         * @brief 重置
         */
        void reset();

        /**
         * @return 模型是否稳定
         */
        bool stable();

    private:
        /**
         * @brief 读取参数配置文件
         * @param file_path 配置文件路径
         */
        void setParam(const std::string &file_path);

        /**
         * @brief 获取先验状态
         * @param X Eigen::Matrix<double, 7, 1> 上一时刻状态
         * @return Eigen::Matrix<double, 7, 1> 先验状态
         */
        Eigen::MatrixXd getTransformation(const Eigen::MatrixXd &X);

        /**
         * @brief 获取状态转移偏导矩阵
         * @param X Eigen::Matrix<double, 7, 1> 先验状态
         * @return Eigen::Matrix<double, 7, 7> 状态转移偏导矩阵
         */
        Eigen::MatrixXd getTransformDP(const Eigen::MatrixXd &X);

        /**
         * @brief 获取状态转移噪声矩阵
         * @param X Eigen::Matrix<double, 7, 1> 先验状态
         * @return Eigen::Matrix<double, 7, 7> 状态转移噪声矩阵
         */
        Eigen::MatrixXd getTransformNoise(const Eigen::MatrixXd &X);

        /**
         * @brief 获取先验观测量
         * @param X Eigen::Matrix<double, 7, 1> 先验状态
         * @param i 标准装甲板索引
         * @return Eigen::Matrix<double, 4, 1> 先验观测矩阵
         */
        Eigen::MatrixXd getPredictiveMeasurement(const Eigen::MatrixXd &X, int i);

        /**
         * @brief 获取观测方程偏导矩阵
         * @param X Eigen::Matrix<double, 7, 1> 先验状态
         * @param i 标准装甲板索引
         * @return Eigen::Matrix<double, 4, 7> 观测偏导矩阵
         */
        Eigen::MatrixXd getMeasurementPD(const Eigen::MatrixXd &X, int i);

        /**
         * @brief 获取观测噪声偏导矩阵
         * @param X Eigen::Matrix<double, 7, 1> 先验状态
         * @param i 标准装甲板索引
         * @return Eigen::Matrix<double, 4, 4> 观测偏导矩阵
         */
        Eigen::MatrixXd getMeasureNoisePD(const Eigen::MatrixXd &X, int i);

    private:
        Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> m_F;          // 状态转移偏导矩阵
    };

    class OutpostObserver : public ModelObserver
    {
    public:
        OutpostObserver();
        ~OutpostObserver();

        /**
         * @brief 先验预测
         * @param armors 绝对坐标系装甲板序列
         * @return std::pair<Eigen::Matrix<double, 10, 1>, Eigen::Matrix<double, 10, 10>>(X_, P_)
         */
        std::pair<Eigen::MatrixXd, Eigen::MatrixXd> predict(const Armors &armors);

        /**
         * @brief 匹配装甲板序列和标准装甲板
         * @param armors 绝对坐标系装甲板序列
         * @param X_ Eigen::Matrix<double, 10, 1> 先验状态
         * @param P_ Eigen::Matrix<double, 10, 10> 先验状态协方差
         * @param match 装甲板关联
         * @return 运动状态
         */
        std::shared_ptr<KinematicModel> update(const Armors &armors, const Eigen::MatrixXd &X_, const Eigen::MatrixXd &P_, std::map<int, int> &match);

        /**
         * @brief 重置
         */
        void reset();

        /**
         * @return 模型是否稳定
         */
        bool stable();

    private:
        /**
         * @brief 读取参数配置文件
         * @param file_path 配置文件路径
         */
        void setParam(const std::string &file_path);

        /**
         * @brief 获取先验观测量
         * @param X Eigen::Matrix<double, 10, 1> 先验状态
         * @param i 标准装甲板索引
         * @return Eigen::Matrix<double, 4, 1> 先验观测矩阵
         */
        Eigen::MatrixXd getPredictiveMeasurement(const Eigen::MatrixXd &X, int i);

        /**
         * @brief 获取观测方程偏导矩阵
         * @param X Eigen::Matrix<double, 10, 1> 先验状态
         * @param i 标准装甲板索引
         * @return Eigen::Matrix<double, 4, 10> 观测偏导矩阵
         */
        Eigen::MatrixXd getMeasurementPD(const Eigen::MatrixXd &X, int i);

        /**
         * @brief 获取观测噪声偏导矩阵
         * @param X Eigen::Matrix<double, 10, 1> 先验状态
         * @param i 标准装甲板索引
         * @return Eigen::Matrix<double, 4, 4> 观测偏导矩阵
         */
        Eigen::MatrixXd getMeasureNoisePD(const Eigen::MatrixXd &X, int i);
        
        double m_process_noise[3];
    };

}   // wmj
