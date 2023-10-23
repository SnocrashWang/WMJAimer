#pragma once

#include "../../../libControl/Pose/include/AngleSolver.hpp"

#include "KinematicModel.hpp"
#include "ModelObserver.hpp"
#include "MPC.hpp"

// #define STANDARD
// #define BALANCE

// 由于本项目未包含响应依赖库，此处忽略ROI计算部分的编译
#define PARTIAL_COMPILE


namespace wmj
{
    /**
     * @brief 利用EKF对整车状态进行建模并预测
     * @author 王铭远
     */
    class Aimer
    {
    public:
        Aimer();
        ~Aimer();

        /**
         * @brief 模块主接口。
         * @param armors 相机坐标系装甲板序列
         * @param cur_pose 当前位姿
         * @param bullet_speed 当前射速
         * @param control_mode 0 返回目标位姿，1 返回目标速度
         * @return 云台的目标位姿或者目标速度
         */
        GimbalPose getTargetPose(const Armors &armors, const GimbalPose &cur_pose, double bullet_speed, int control_mode = 1);

        /**
         * @brief 数据解算核心过程，同时也是提供给数据模拟器的主接口
         * @param armors 装甲板序列
         * @return 运动学状态
         */
        std::shared_ptr<KinematicModel> resolve(const Armors &armors);

        /**
         * @brief 完全重置
         */
        void reset();

        /**
         * @return 是否允许控制
         */
        bool isReady();

        /**
         * @return 是否允许击发
         */
        bool shootable();

        /**
         * @brief 抛弃装甲板序列中数据异常者
         * @param armors 用于处理的装甲板序列
         */
        void errorHandling(Armors &armors);

        /** 
         * @brief 强制指定运动学模型并实例化，慎用
         */
        void setModelType(KinematicModel::Type type);

    private:
        /**
         * @brief 读取参数配置文件
         * @param file_path 配置文件路径
         */
        void setParam(const std::string &file_path);

        /**
         * @brief 根据熵权法获取装甲板相关性矩阵，相关性指标为负向指标
         * @param armors 识别装甲板序列
         * @param status 当前运动状态
         * @return 相关性矩阵
         */
        Eigen::MatrixXd getScoreMat(const Armors &detect_armors, const Armors &standard_armors);

        /**
         * @brief 装甲板匹配
         * @param score 相关性矩阵。相关性指标为负向指标
         * @return 装甲板关联。键为识别装甲板索引，值为标准装甲板索引
         */
        std::map<int, int> match(const Eigen::MatrixXd &score);

        bool m_debug;
        bool m_ekf_on;                          // 是否使用EKF
        bool m_predict_on;                      // 是否使用预测
        bool m_track_center;                    // 是否跟随中心
        bool m_tracking;                        // 是否跟随
        bool m_enable_shoot;                    // 是否允许自动击发

        int m_tracked_ID;                       // 当前锁定ID
        int m_type_init_cnt;                    // 模型判断识别计数
        int m_track_lost_cnt;                   // 完全丢识别计数
        int m_center_tracked;                   // 锁中心状态标志位
        int m_all_white_cnt;                    // 连续识别到全部为白色装甲板的次数
        double m_next_shoot_time;               // 下次允许射击时间

        // 参数
        double m_time_off;                      // 预测时间补偿
        double m_switch_threshold;              // 更新装甲板切换的角度阈值，角度制
        double m_init_pose_tolerance;           // 初始化位姿变化最大值，角度制
        double m_rubbish_data_tolerance;        // 可接受作为输入数据的装甲板在相机系的最大角度，角度制
        double m_force_aim_palstance_threshold; // 强制允许发射的目标旋转速度最大值，弧度制
        double m_aim_center_palstance_threshold;// 跟随圆心转跟随装甲板的目标旋转速度最大值，弧度制
        double m_switch_trackmode_threshold;    // 更换锁中心模式角速度阈值，弧度制
        double m_aim_angle_tolerance;           // 自动击发时目标装甲板相对偏角最大值，角度制
        double m_aim_pose_tolerance;            // 自动击发位姿偏差最大值，弧度制
        double m_aim_center_angle_tolerance;    // 跟随圆心自动击发目标偏角判断，角度制
        double m_score_tolerance;               // 装甲板匹配得分最大值
        int m_all_white_tolerance_stop_shoot;   // 连续识别到全部为白色装甲板的次数容忍度，将会停止发射
        int m_all_white_tolerance_reset;        // 连续识别到全部为白色装甲板的次数容忍度，将会重置整个模块
        double m_shoot_interval;                // 发射间隔，单位：秒

        GimbalPose m_cur_pose;                  // 当前位姿
        GimbalPose m_target_pose;               // 目标位姿

        KinematicModel::Type m_model_type;                  // 模型类型
        std::shared_ptr<KinematicModel> m_status;           // 当前状态。为了保留最佳目标装甲板，必须声明为类成员变量以保留其索引值
        std::shared_ptr<ModelObserver> m_model_observer;    // 状态观测器
        AngleSolver m_angle_solver;
    
    public:
        std::shared_ptr<MPC> m_MPC;                         // 模型预测控制器

    private:
        /**
         * @brief 根据相似度矩阵枚举最小代价匹配
         * @author 赵亮程
         */
        class Match
        {
        public:
            Match();
            ~Match();

            /**
             * @brief 获取矩阵中的一组位于互不相同的行和列的数的位置，且选取的这些数的和最小
             * @param matrix 输入的矩阵
             * @param n 匹配装甲板数
             * @return map<int, int> 行和列的位置
             */
            std::map<int, int> getMatch(Eigen::MatrixXd matrix, double score_max, int m);

            bool debug = false;     // 是否输出生成的所有组合数

        private:
            /**
             * @brief 求出在n个数中选取k个数，所有的组合结果，即C(n,k)的所有结果
             * @param input 一组数，由用户决定
             * @param tmp_v 存储中间结果
             * @param result C(n,k)结果
             * @param start 起始位置，应指定为0
             * @param k k个数，由用户决定
             */
            void getCombinationsNumbers(std::vector<int> &input, std::vector<int> &tmp_v, std::vector<std::vector<int>> &result, int start, int k);

            double min = 0, tmp = 0;                    // 最小值

            std::vector<int> col;                       // 数列中的每个数代表矩阵的每一列
            std::vector<int> row;                       // 数列中的每个数代表矩阵的每一行
            std::vector<int> tmp_v;                     // 存储C(n,k)的中间结果

            std::vector<std::vector<int>> result;       // 存储C(n,k)的结果
            std::vector<std::vector<int>> nAfour;       // 存储A(n,4)的结果
            std::vector<std::vector<int>> fourAfour;    // 存储A(4,4)的结果

            std::map<int, int> row_col;                 // 存储最终结果，row_col[i]=j表示矩阵的第i行第j列是要选取的数

        } m_match;

#ifndef PARTIAL_COMPILE
    /************ ROI ************
     * @brief 控制重投影ROI回传
     * @author 王云飞
     */
    public:
        /**
         * @brief 设置深度学习输入图像尺寸
         */
        void setDeepROISize(cv::Size2i deep_roi_size);

        /**
         * @brief 回传左目ROI区域
         */
        cv::Rect2d getLeftROI();
        /**
         * @brief 回传右目ROI区域
         */
        cv::Rect2d getRightROI();

        /**
         * @brief 绘制重投影点
         * @param src 画布
         * @param cur_pose 当前位姿
         */
	    void drawReProjectPoint(cv::Mat &src);

    private:
        /**
         * @brief 传入相机内参矩阵（使用双目时传入左相机参数）
         */
        void setCameraParam();

        /**
         * @brief 内部计算主函数。计算ROI
         */
        void setROI(std::shared_ptr<KinematicModel> status, const Armors &armors);

        /**
         * @brief 获取重投影点
         */
        cv::Point2f getReProjectPoint(const cv::Point3f &point);

        std::shared_ptr<SelectorParam> m_roi_params;    // ROI参数
        DeepROISizeState m_deep_roi_state;              // 深度学习ROI标准
        cv::Rect2d m_deep_default_roi;
        cv::Mat m_camera_mat;

        int m_min_roi_height;
        float m_roi_height_zoom_rate;
        float m_roi_width_zoom_rate;

        cv::Rect2d m_return_roi_left;       // 左目ROI回传
        cv::Rect2d m_return_roi_right;      // 右目ROI回传
    /************ ROI ************/
#endif
    };

}   // wmj
