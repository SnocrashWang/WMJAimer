#include "../include/MPC.hpp"

namespace wmj
{
    MPC::MPC()
    {
        setParam(AIM_CFG);
        setMPCMatrix();
    }
    MPC::~MPC()
    {}

    // TODO 等MPC优越性验证通过后或可考虑调整参数位置，现在有点乱
    void MPC::setParam(const std::string &file_path)
    {
        cv::FileStorage fs(file_path, cv::FileStorage::READ);
        fs["MPC"]["debug"]              >> m_debug;
        fs["Aimer"]["track_center"]     >> m_track_center;

        fs["MPC"]["step"]                                                   >> m_step;
        fs["ModelObserver"]["dt"]                                           >> m_dt;
        fs["Aimer"]["time_off"]                                             >> m_time_off;
        fs["Aimer"]["switch_threshold"]                                     >> m_switch_threshold;
        fs["Aimer"]["TRACK_CENTER"]["aim_center_palstance_threshold"]       >> m_aim_center_palstance_threshold;
        fs["Aimer"]["TRACK_CENTER"]["switch_trackmode_threshold"]           >> m_switch_trackmode_threshold;

        for (int i = 0; i < _D_STATE_; i++)
        {
            fs["MPC"]["process_error_weight"]["Q" + std::to_string(i)]      >> m_process_error_weight[i];
            fs["MPC"]["target_error_weight"]["F" + std::to_string(i)]       >> m_target_error_weight[i];
        }
        for (int i = 0; i < _D_CTRL_; i++)
        {
            fs["MPC"]["process_control_weight"]["R" + std::to_string(i)]    >> m_process_control_weight[i];
        }

        fs.release();
    }

    void MPC::setMPCMatrix()
    {
        // 初始化系统参数
        m_A.resize(_D_STATE_, _D_STATE_);
        m_B.resize(_D_STATE_, _D_CTRL_);
        m_Q.resize(_D_STATE_, _D_STATE_);
        m_R.resize(_D_CTRL_, _D_CTRL_);
        m_F.resize(_D_STATE_, _D_STATE_);

#if _D_STATE_ == 2
        m_A << 1, 0,
               0, 1;
        m_B << m_dt, 0,
               0,    m_dt;
#elif _D_STATE_ == 4
        m_A << 1, 0, m_dt, 0,
               0, 1, 0,    m_dt,
               0, 0, 0,    0,
               0, 0, 0,    0;
        m_B << 0, 0,
               0, 0,
               1, 0,
               0, 1;
#endif
        m_Q = m_process_error_weight.asDiagonal();
        m_R = m_process_control_weight.asDiagonal();
        m_F = m_target_error_weight.asDiagonal();

        // if (m_debug)
        // {
        //     std::cout << "[MPC] A: " << std::endl << m_A << std::endl;
        //     std::cout << "[MPC] B: " << std::endl << m_B << std::endl;
        //     std::cout << "[MPC] Q: " << std::endl << m_Q << std::endl;
        //     std::cout << "[MPC] R: " << std::endl << m_R << std::endl;
        //     std::cout << "[MPC] F: " << std::endl << m_F << std::endl;
        // }

        // 初始化中间量
        Eigen::MatrixXd M;
        Eigen::MatrixXd C;
        M.resize((m_step + 1) * _D_STATE_, _D_STATE_);
        C.resize((m_step + 1) * _D_STATE_, m_step * _D_STATE_);
        M.setZero();
        C.setZero();

        Eigen::MatrixXd tmp = Eigen::Matrix<double, _D_STATE_, _D_STATE_>::Identity();
        M.block<_D_STATE_, _D_STATE_>(0, 0) = Eigen::Matrix<double, _D_STATE_, _D_STATE_>::Identity();
        for (int i = 1; i <= m_step; i++)
        {
            C.block<_D_STATE_, _D_CTRL_>(i * _D_STATE_, 0) = tmp * m_B;
            for (int j = 1; j < m_step; j++)
            {
                C.block<_D_STATE_, _D_CTRL_>(i * _D_STATE_, j * _D_CTRL_) = C.block<_D_STATE_, _D_CTRL_>((i - 1) * _D_STATE_, (j - 1) * _D_CTRL_);
            }
            tmp *= m_A;
            M.block<_D_STATE_, _D_STATE_>(i * _D_STATE_, 0) = tmp;
        }

        // if (m_debug)
        // {
        //     std::cout << "[MPC] M: " << std::endl << M << std::endl;
        //     std::cout << "[MPC] C: " << std::endl << C << std::endl;
        // }
        
        Eigen::MatrixXd Q_;
        Eigen::MatrixXd R_;
        Q_.resize((m_step + 1) *_D_STATE_, (m_step + 1) * _D_STATE_);
        R_.resize(m_step *_D_CTRL_, m_step * _D_CTRL_);
        Q_.setZero();
        R_.setZero();

        for (int i = 0; i < m_step; i++)
        {
            Q_.block<_D_STATE_, _D_STATE_>(i * _D_STATE_, i * _D_STATE_) = m_Q;
            R_.block<_D_CTRL_, _D_CTRL_>(i * _D_CTRL_, i * _D_CTRL_) = m_R;
        }
        Q_.block<_D_STATE_, _D_STATE_>(m_step * _D_STATE_, m_step * _D_STATE_) = m_F;

        // if (m_debug)
        // {
        //     std::cout << "[MPC] Q_: " << std::endl << Q_ << std::endl;
        //     std::cout << "[MPC] R_: " << std::endl << R_ << std::endl;
        // }

        // 计算损失函数系数
        m_H = C.transpose() * Q_ * C + R_;
        m_E = C.transpose() * Q_ * M;
        m_Ed = C.transpose() * Q_;

        // if (m_debug)
        // {
        //     std::cout << "[MPC] H: " << std::endl << m_H << std::endl;
        //     std::cout << "[MPC] E: " << std::endl << m_E << std::endl;
        //     std::cout << "[MPC] Ed: " << std::endl << m_Ed << std::endl;
        // }
    }

    GimbalPose MPC::getGimbalSpeed(std::shared_ptr<KinematicModel> status, const GimbalPose cur_pose, double bullet_speed)
    {
        m_bullet_speed = bullet_speed;
        m_cur_pose = cur_pose;

        // 根据运动模型预测控制目标
        Eigen::MatrixXd predict_ctrl_target = getPredictControlTarget(status);
        // predict_ctrl_target.conservativeResize(predict_ctrl_target.rows() * predict_ctrl_target.cols(), 1);
        predict_ctrl_target.resize(predict_ctrl_target.rows() * predict_ctrl_target.cols(), 1);
        // std::cout << "[MPC] Predict_Ctrl_Target: " << std::endl << predict_ctrl_target << std::endl;

        // 求解最优控制
        Eigen::Matrix<double, 2, 1> cur_state;
        cur_state << m_cur_pose.pitch, m_cur_pose.yaw;
        Eigen::MatrixXd U = solveQuadProg(m_H, m_E * cur_state - m_Ed * predict_ctrl_target);
        GimbalPose target_ctrl(U(0, 0), U(1, 0));

        return target_ctrl;
    }

    void MPC::reset()
    {
        m_center_tracked = false;
    }

    Eigen::Matrix<double, _D_STATE_, Eigen::Dynamic> MPC::getPredictControlTarget(std::shared_ptr<KinematicModel> status)
    {
        // 控制目标
        Eigen::MatrixXd predict_ctrl_target;
        predict_ctrl_target.resize(_D_STATE_, m_step + 1);
        predict_ctrl_target.setZero();

        // 判断是否锁中心
        if (m_track_center && ((abs(status->palstance) - m_switch_trackmode_threshold) > m_aim_center_palstance_threshold))
        {
            m_center_tracked = true;
        }
        else
        {
            m_center_tracked = false;
        }

        for (int i = 0; i <= m_step; i++)
        {
            GimbalPose pose = getTargetPose(status, m_dt * i);
            predict_ctrl_target(0, i) = pose.pitch;
            predict_ctrl_target(1, i) = pose.yaw;
        }
        // 若状态量包含速度
        if (_D_STATE_ == 4)
        {
            for (int i = 0; i < m_step; i++)
            {
                predict_ctrl_target(2, i) = (predict_ctrl_target(0, i + 1) - predict_ctrl_target(0, i)) / m_dt;
                predict_ctrl_target(3, i) = (predict_ctrl_target(1, i + 1) - predict_ctrl_target(1, i)) / m_dt;
            }
            GimbalPose pose = getTargetPose(status, m_dt * (m_step + 1));
            predict_ctrl_target(2, m_step) = (pose.pitch - predict_ctrl_target(0, m_step)) / m_dt;
            predict_ctrl_target(3, m_step) = (pose.yaw - predict_ctrl_target(1, m_step)) / m_dt;
        }
        if (m_debug)
            std::cout << "[MPC] predict_ctrl_target: " << std::endl << predict_ctrl_target << std::endl;

        return predict_ctrl_target;
    }

    GimbalPose MPC::getTargetPose(std::shared_ptr<KinematicModel> status, double time_diff)
    {
        double hit_time = time_diff;
        Armor abs_target_armor;
        // 根据是否锁中心确定目标位姿
        if (m_center_tracked)
        {
            hit_time += getDistance(status->getFacingArmor(hit_time).m_position) / m_bullet_speed + m_time_off;
            abs_target_armor = status->getFacingArmor(hit_time);
        }
        else
        {
            hit_time += getDistance(status->getClosestArmor(hit_time, 0).m_position) / m_bullet_speed + m_time_off;
            abs_target_armor = status->getClosestArmor(hit_time, m_switch_threshold);
        }

        return m_angle_solver.getAngle(m_angle_solver.abs2cam(abs_target_armor.m_position, m_cur_pose), m_cur_pose, m_bullet_speed);
    }

    Eigen::MatrixXd MPC::solveQuadProg(Eigen::MatrixXd H, Eigen::MatrixXd f)
    {
        // 迭代中间量，在循环中可以看作是上一次迭代的结果
        Eigen::MatrixXd x0;
        Eigen::MatrixXd r0;
        Eigen::MatrixXd p0;
        x0 = f;
        x0.setZero();
        r0 = H * x0 + f;
        p0 = r0;

        // 迭代更新量
        Eigen::MatrixXd x;
        Eigen::MatrixXd r;
        Eigen::MatrixXd p;
        Eigen::MatrixXd alpha;
        Eigen::MatrixXd beta;

        for (int i = 0; i < 1000; i++)
        {
            alpha = (r0.transpose() * r0) * (p0.transpose() * H * p0).inverse();
            x = x0 - alpha(0, 0) * p0;
            r = r0 - alpha(0, 0) * H * p0;
            beta = (r.transpose() * r) * (r0.transpose() * r0).inverse();
            p = r + beta(0, 0) * p0;

            if ((x - x0).norm() < 1e-8)
            {
                if (m_debug)
                    std::cout << "[MPC] quadratic_result: " << std::endl << x.transpose() << std::endl;

                return x;
            }

            x0 = x;
            r0 = r;
            p0 = p;
        }

        std::cout << _warning("[MPC] Failed to solve the quadratic programming in limited times") << std::endl;
        return x;
    }

}   // wmj
