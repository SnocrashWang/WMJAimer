#include "../include/ModelObserver.hpp"

namespace wmj
{
    ModelObserver::ModelObserver()
    {}
    ModelObserver::~ModelObserver()
    {}


    StandardObserver::StandardObserver()
    :   ModelObserver()
    {
        setParam(AIM_CFG);

        reset();

        m_F << 1, m_dt, 0, 0,    0, 0, 0, 0, 0, 0,
               0, 1,    0, 0,    0, 0, 0, 0, 0, 0,
               0, 0,    1, m_dt, 0, 0, 0, 0, 0, 0,
               0, 0,    0, 1,    0, 0, 0, 0, 0, 0,
               0, 0,    0, 0,    1, 0, 0, 0, 0, 0,
               0, 0,    0, 0,    0, 1, 0, 0, 0, 0,
               0, 0,    0, 0,    0, 0, 1, 0, 0, 0,
               0, 0,    0, 0,    0, 0, 0, 1, 0, 0,
               0, 0,    0, 0,    0, 0, 0, 0, 1, m_dt,
               0, 0,    0, 0,    0, 0, 0, 0, 0, 1;

        double dd = m_process_noise[0];
        double da = m_process_noise[1];
        double dz = m_process_noise[2];
        double dr = m_process_noise[3];
        double t4 = pow(m_dt, 3) / 3;
        double t3 = pow(m_dt, 2) / 2;
        double t2 = pow(m_dt, 1);
        m_Q << t4 * dd, t3 * dd, 0,       0,       0,  0, 0,  0,  0,       0,
               t3 * dd, t2 * dd, 0,       0,       0,  0, 0,  0,  0,       0,
               0,       0,       t4 * dd, t3 * dd, 0,  0, 0,  0,  0,       0,
               0,       0,       t3 * dd, t2 * dd, 0,  0, 0,  0,  0,       0,
               0,       0,       0,       0,       dz, 0, 0,  0,  0,       0,
               0,       0,       0,       0,       0, dz, 0,  0,  0,       0,
               0,       0,       0,       0,       0,  0, dr, 0,  0,       0,
               0,       0,       0,       0,       0,  0, 0,  dr, 0,       0,
               0,       0,       0,       0,       0,  0, 0,  0,  t4 * da, t3 * da,
               0,       0,       0,       0,       0,  0, 0,  0,  t3 * da, t2 * da;

        Eigen::VectorXd measurement_noise4(4);
        measurement_noise4 << m_measure_noise[0],
                              m_measure_noise[0],
                              m_measure_noise[1],
                              m_measure_noise[2];
        m_R = measurement_noise4.asDiagonal();
        Eigen::VectorXd measurement_noise8(8);
        measurement_noise8 << measurement_noise4,
                              measurement_noise4;
        m_RR = measurement_noise8.asDiagonal();
    }
    StandardObserver::~StandardObserver()
    {}

    void StandardObserver::setParam(const std::string &file_path)
    {
        cv::FileStorage fs(file_path, cv::FileStorage::READ);
        fs["ModelObserver"]["debug"]    >> m_debug;
        fs["ModelObserver"]["dt"]       >> m_dt;

        fs["ModelObserver"]["Standard"]["init_radius"]                          >> m_init_radius;
        fs["ModelObserver"]["Standard"]["gain"]                                 >> m_gain;

        fs["ModelObserver"]["Standard"]["process_noise"]["displace_high_diff"]  >> m_process_noise[0];
        fs["ModelObserver"]["Standard"]["process_noise"]["anglar_high_diff"]    >> m_process_noise[1];
        fs["ModelObserver"]["Standard"]["process_noise"]["height"]              >> m_process_noise[2];
        fs["ModelObserver"]["Standard"]["process_noise"]["radius"]              >> m_process_noise[3];

        fs["ModelObserver"]["Standard"]["measure_noise"]["pose"]                >> m_measure_noise[0];
        fs["ModelObserver"]["Standard"]["measure_noise"]["distance"]            >> m_measure_noise[1];
        fs["ModelObserver"]["Standard"]["measure_noise"]["angle"]               >> m_measure_noise[2];
        fs.release();

        m_X.resize(10, 1);
        m_X_update.resize(10, 1);
        m_P.resize(10, 10);
        m_F.resize(10, 10);
        m_Q.resize(10, 10);
        m_R.resize(4, 4);
        m_RR.resize(8, 8);
    }

    void StandardObserver::reset()
    {
        m_init = false;

        m_P = Eigen::Matrix<double, 10, 10>::Identity() * 1e-5;
    }

    std::pair<Eigen::MatrixXd, Eigen::MatrixXd> StandardObserver::predict(const Armors &armors)
    {
        if (!m_init)
        {
            for (auto armor : armors)
            {
                // 取排序后第一块非白色装甲板计算初始化状态
                // 如果进入该初始化部分，前置逻辑已保证序列中一定有一个非白色装甲板
                if (armor.m_color != _COLOR::_WHITE)
                {
                    m_X << armor.m_position.x - m_init_radius * cos(armor.m_yaw_angle),
                           0,
                           armor.m_position.y - m_init_radius * sin(armor.m_yaw_angle),
                           0,
                           armor.m_position.z,
                           armor.m_position.z,
                           m_init_radius,
                           m_init_radius,
                           armor.m_yaw_angle,
                           0;
                    m_init = true;
                    break;
                }
            }
        }

        // 预测
        Eigen::MatrixXd X_ = m_F * m_X;
        Eigen::MatrixXd P_ = m_F * m_P * m_F.transpose() + m_Q;
        if (m_debug)
        {
            std::cout << "[StandardObserver] X_: " << std::endl << X_ << std::endl;
            std::cout << "[StandardObserver] P_: " << std::endl << P_ << std::endl;
        }

        return std::make_pair(X_, P_);
    }

    std::shared_ptr<KinematicModel> StandardObserver::update(const Armors &armors, const Eigen::MatrixXd &X_, const Eigen::MatrixXd &P_, std::map<int, int> &match)
    {
        // 观测
        Eigen::MatrixXd Z;      // 实际观测量
        Eigen::MatrixXd h;      // 先验观测量
        Eigen::MatrixXd H;      // 观测方程偏导矩阵
        Eigen::MatrixXd V;      // 观测噪声偏导矩阵
        Eigen::MatrixXd K;      // 置信度权重矩阵
        Eigen::MatrixXd R;      // 时变观测噪声矩阵

        if (match.size() == 1)
        {
            int num = match.begin()->first;
            Z.resize(4, 1);
            Z << armors[num].m_position.x, armors[num].m_position.y, armors[num].m_position.z, armors[num].m_yaw_angle;
            h.resize(4, 1);
            h << getPredictiveMeasurement(X_, match[num]);
            H.resize(4, 10);
            H << getMeasurementPD(X_, match[num]);
            V.resize(4, 4);
            V << getMeasureNoisePD(X_, match[num]);
            R = m_R;
            R(2, 2) *= m_gain * abs(Z(3, 0));

            K = P_ * H.transpose() * ((H * P_ * H.transpose() + V * R * V.transpose())).inverse();
        }
        else if (match.size() == 2)
        {
            int num1 = match.begin()->first;
            int num2 = (++match.begin())->first;
            Z.resize(8, 1);
            Z << armors[num1].m_position.x, armors[num1].m_position.y, armors[num1].m_position.z, armors[num1].m_yaw_angle,
                 armors[num2].m_position.x, armors[num2].m_position.y, armors[num2].m_position.z, armors[num2].m_yaw_angle;
            h.resize(8, 1);
            h << getPredictiveMeasurement(X_, match[num1]),
                 getPredictiveMeasurement(X_, match[num2]);
            H.resize(8, 10);
            H << getMeasurementPD(X_, match[num1]),
                 getMeasurementPD(X_, match[num2]);
            V.resize(8, 8);
            V << getMeasureNoisePD(X_, match[num1]), Eigen::MatrixXd::Zero(4, 4),
                 Eigen::MatrixXd::Zero(4, 4), getMeasureNoisePD(X_, match[num2]);
            R = m_RR;
            R(2, 2) *= m_gain * abs(Z(3, 0));
            R(6, 6) *= m_gain * abs(Z(7, 0));

            K = P_ * H.transpose() * ((H * P_ * H.transpose() + V * R * V.transpose())).inverse();
        }
        else
        {
            // 若匹配数目不为1或2，则认为丢识别或者误判，直接返回先验状态
            m_X = X_;
            m_P = P_;

            return std::make_shared<StandardModel>(m_X);
        }

        // if (m_debug)
        // {
        //     std::cout << "[StandardObserver] Z: " << std::endl << Z << std::endl;
        //     std::cout << "[StandardObserver] h: " << std::endl << h << std::endl;
        //     std::cout << "[StandardObserver] H: " << std::endl << H << std::endl;
        // }

        // 更新
        Eigen::MatrixXd tmp = Z - h;
        for (int i = 0; i < match.size(); i++)
        {
            tmp(3 + i * 4, 0) = _std_radian(tmp(3 + i * 4, 0));
        }
        m_X_update = K * tmp;
        if (m_debug)
        {
            std::cout << "[StandardObserver] X_update: " << std::endl << m_X_update << std::endl;
        }

        m_X = X_ + m_X_update;
        m_P = (Eigen::Matrix<double, 10, 10>::Identity() - K * H) * P_;

        if (m_debug)
        {
            std::cout << "[StandardObserver] X:" << std::endl << m_X << std::endl;
            std::cout << "[StandardObserver] P:" << std::endl << m_P << std::endl;
        }

        return std::make_shared<StandardModel>(m_X);
    }

    bool StandardObserver::stable()
    {
        return true;
    }

    Eigen::MatrixXd StandardObserver::getPredictiveMeasurement(const Eigen::MatrixXd &X, int i)
    {
        Eigen::Matrix<double, 4, 1> h;
        h << X(0, 0) + X(6 + i % 2, 0) * cos(X(8, 0) + i * PI / 2),
             X(2, 0) + X(6 + i % 2, 0) * sin(X(8, 0) + i * PI / 2),
             X(4 + i % 2, 0),
             X(8, 0) + i * PI / 2;
        return h;
    }

    Eigen::MatrixXd StandardObserver::getMeasurementPD(const Eigen::MatrixXd &X, int i)
    {
        Eigen::Matrix<double, 4, 10> H;
        H << 1, 0, 0, 0, 0,           0,     ((i + 1) % 2) * cos(X(8, 0) + i * PI / 2), (i % 2) * cos(X(8, 0) + i * PI / 2),-X(6 + i % 2, 0) * sin(X(8, 0) + i * PI / 2), 0,
             0, 0, 1, 0, 0,           0,     ((i + 1) % 2) * sin(X(8, 0) + i * PI / 2), (i % 2) * sin(X(8, 0) + i * PI / 2), X(6 + i % 2, 0) * cos(X(8, 0) + i * PI / 2), 0,
             0, 0, 0, 0, (i + 1) % 2, i % 2, 0,                                         0,                                   0,                                           0,
             0, 0, 0, 0, 0,           0,     0,                                         0,                                   1,                                           0;
        return H;
    }

    Eigen::MatrixXd StandardObserver::getMeasureNoisePD(const Eigen::MatrixXd &X, int i)
    {
        Eigen::Matrix<double, 4, 1> h = getPredictiveMeasurement(X, i);
        double psi = atan2(h(1, 0), h(0, 0));
        double phi = atan2(h(2, 0), sqrt(pow(h(0, 0), 2) + pow(h(1, 0), 2)));
        double d = sqrt(pow(h(0, 0), 2) + pow(h(1, 0), 2) + pow(h(2, 0), 2));
        Eigen::Matrix<double, 4, 4> V;
        V << -d * cos(phi) * sin(psi), -d * sin(phi) * cos(psi), cos(phi) * cos(psi), 0,
              d * cos(phi) * cos(psi), -d * sin(phi) * sin(psi), cos(phi) * sin(psi), 0,
              0,                        d * cos(phi),            sin(phi),            0,
              0,                        0,                       0,                   1;
        return V;
    }


    BalanceObserver::BalanceObserver()
    :   ModelObserver()
    {
        setParam(AIM_CFG);

        reset();

        Eigen::VectorXd measurement_noise4(4);
        measurement_noise4 << m_measure_noise[0],
                              m_measure_noise[0],
                              m_measure_noise[1],
                              m_measure_noise[2];
        m_R = measurement_noise4.asDiagonal();
    }
    BalanceObserver::~BalanceObserver()
    {}

    void BalanceObserver::setParam(const std::string &file_path)
    {
        cv::FileStorage fs(file_path, cv::FileStorage::READ);
        fs["ModelObserver"]["debug"]    >> m_debug;
        fs["ModelObserver"]["dt"]       >> m_dt;

        fs["ModelObserver"]["Balance"]["init_radius"]                          >> m_init_radius;
        fs["ModelObserver"]["Balance"]["gain"]                                 >> m_gain;

        fs["ModelObserver"]["Balance"]["process_noise"]["displace_high_diff"]  >> m_process_noise[0];
        fs["ModelObserver"]["Balance"]["process_noise"]["anglar_high_diff"]    >> m_process_noise[1];
        fs["ModelObserver"]["Balance"]["process_noise"]["height"]              >> m_process_noise[2];
        fs["ModelObserver"]["Balance"]["process_noise"]["radius"]              >> m_process_noise[3];

        fs["ModelObserver"]["Balance"]["measure_noise"]["pose"]                >> m_measure_noise[0];
        fs["ModelObserver"]["Balance"]["measure_noise"]["distance"]            >> m_measure_noise[1];
        fs["ModelObserver"]["Balance"]["measure_noise"]["angle"]               >> m_measure_noise[2];
        fs.release();

        m_X.resize(7, 1);
        m_X_update.resize(7, 1);
        m_P.resize(7, 7);
        m_F.resize(7, 7);
        m_Q.resize(7, 7);
        m_R.resize(4, 4);
    }

    void BalanceObserver::reset()
    {
        m_init = false;

        m_P = Eigen::Matrix<double, 7, 7>::Identity() * 1e-5;
    }

    std::pair<Eigen::MatrixXd, Eigen::MatrixXd> BalanceObserver::predict(const Armors &armors)
    {
        if (!m_init)
        {
            for (auto armor : armors)
            {
                // 取排序后第一块非白色装甲板计算初始化状态
                // 如果进入该初始化部分，前置逻辑已保证序列中一定有一个非白色装甲板
                if (armor.m_color != _COLOR::_WHITE)
                {
                    m_X << armor.m_position.x - m_init_radius * cos(armor.m_yaw_angle),
                           armor.m_position.y - m_init_radius * sin(armor.m_yaw_angle),
                           0,
                           armor.m_position.z,
                           m_init_radius,
                           armor.m_yaw_angle,
                           0;
                    m_init = true;
                    break;
                }
            }
        }

        // 预测
        Eigen::MatrixXd X_ = getTransformation(m_X);
        m_F = getTransformDP(m_X);
        m_Q = getTransformNoise(m_X);
        Eigen::MatrixXd P_ = m_F * m_P * m_F.transpose() + m_Q;
        if (m_debug)
        {
            std::cout << "[BalanceObserver] X_: " << std::endl << X_ << std::endl;
            // std::cout << "[BalanceObserver] P_: " << std::endl << P_ << std::endl;
        }

        return std::make_pair(X_, P_);
    }

    std::shared_ptr<KinematicModel> BalanceObserver::update(const Armors &armors, const Eigen::MatrixXd &X_, const Eigen::MatrixXd &P_, std::map<int, int> &match)
    {
        // 观测
        Eigen::MatrixXd Z;      // 实际观测量
        Eigen::MatrixXd h;      // 先验观测量
        Eigen::MatrixXd H;      // 观测方程偏导矩阵
        Eigen::MatrixXd V;      // 观测噪声偏导矩阵
        Eigen::MatrixXd K;      // 置信度权重矩阵
        Eigen::MatrixXd R;      // 时变观测噪声矩阵

        if (match.size() == 1)
        {
            int num = match.begin()->first;
            Z.resize(4, 1);
            Z << armors[num].m_position.x, armors[num].m_position.y, armors[num].m_position.z, armors[num].m_yaw_angle;
            h.resize(4, 1);
            h << getPredictiveMeasurement(X_, match[num]);
            H.resize(4, 7);
            H << getMeasurementPD(X_, match[num]);
            V.resize(4, 4);
            V << getMeasureNoisePD(X_, match[num]);
            R = m_R;
            R(2, 2) *= m_gain * abs(Z(3, 0));

            K = P_ * H.transpose() * ((H * P_ * H.transpose() + V * R * V.transpose())).inverse();
        }
        else
        {
            // 若匹配数目不为1，则认为丢识别或者误判，直接返回先验状态
            m_X = X_;
            m_P = P_;

            return std::make_shared<BalanceModel>(m_X);
        }

        // if (m_debug)
        // {
        //     std::cout << "[BalanceObserver] Z: " << std::endl << Z << std::endl;
        //     std::cout << "[BalanceObserver] h: " << std::endl << h << std::endl;
        //     std::cout << "[BalanceObserver] H: " << std::endl << H << std::endl;
        // }

        // 更新
        Eigen::MatrixXd tmp = Z - h;
        tmp(3, 0) = _std_radian(tmp(3, 0));
        m_X_update = K * tmp;
        if (m_debug)
        {
            // std::cout << "[BalanceObserver] X_update: " << std::endl << m_X_update << std::endl;
        }

        m_X = X_ + m_X_update;
        m_P = (Eigen::Matrix<double, 7, 7>::Identity() - K * H) * P_;

        if (m_debug)
        {
            std::cout << "[BalanceObserver] X:" << std::endl << m_X << std::endl;
            // std::cout << "[BalanceObserver] P:" << std::endl << m_P << std::endl;
        }

        return std::make_shared<BalanceModel>(m_X);
    }

    bool BalanceObserver::stable()
    {
        return true;
    }

    Eigen::MatrixXd BalanceObserver::getTransformation(const Eigen::MatrixXd &X)
    {
        Eigen::Matrix<double, 7, 1> X_;
        // if (X(6, 0) == 0)
        {
            X_ << X(0, 0) + X(2, 0) * cos(X(5, 0)) * m_dt,
                  X(1, 0) + X(2, 0) * sin(X(5, 0)) * m_dt,
                  X(2, 0),
                  X(3, 0),
                  X(4, 0),
                  X(5, 0) + X(6, 0) * m_dt,
                  X(6, 0);
        }
        // else
        // {
        //     X_ << X(0, 0) + X(2, 0) / X(6, 0) * (sin(X(6, 0) * m_dt) + X(5, 0) - sin(X(5, 0))),
        //           X(1, 0) - X(2, 0) / X(6, 0) * (cos(X(6, 0) * m_dt) - X(5, 0) + cos(X(5, 0))),
        //           X(2, 0),
        //           X(3, 0),
        //           X(4, 0),
        //           X(5, 0) + X(6, 0) * m_dt,
        //           X(6, 0);
        // }
        return X_;
    }

    Eigen::MatrixXd BalanceObserver::getTransformDP(const Eigen::MatrixXd &X)
    {
        Eigen::Matrix<double, 7, 7> F;
        // if (X(6, 0) == 0)
        {
            F << 1, 0, cos(X(5, 0)) * m_dt, 0, 0,-X(2, 0) * sin(X(5, 0)) * m_dt, 0,
                 0, 1, sin(X(5, 0)) * m_dt, 0, 0, X(2, 0) * cos(X(5, 0)) * m_dt, 0,
                 0, 0, 1,                   0, 0, 0,                             0,
                 0, 0, 0,                   1, 0, 0,                             0,
                 0, 0, 0,                   0, 1, 0,                             0,
                 0, 0, 0,                   0, 0, 1,                             m_dt,
                 0, 0, 0,                   0, 0, 0,                             1;
        }
        // else
        // {
        //     double cos_theta = cos(X(5, 0));
        //     double cos_theta_t = cos(X(6, 0) * m_dt + X(5, 0));
        //     double sin_theta = sin(X(5, 0));
        //     double sin_theta_t = sin(X(6, 0) * m_dt + X(5, 0));
        //     F << 1, 0, 1 / X(6, 0) * (sin_theta_t - sin_theta), 0, 0, X(2, 0) / X(6, 0) * (cos_theta_t - cos_theta), X(2, 0) / X(6, 0) * cos_theta_t * m_dt - X(2, 0) / pow(X(6, 0), 2) * (sin_theta_t - sin_theta),
        //          0, 1,-1 / X(6, 0) * (cos_theta_t - cos_theta), 0, 0, X(2, 0) / X(6, 0) * (sin_theta_t - sin_theta), X(2, 0) / X(6, 0) * sin_theta_t * m_dt + X(2, 0) / pow(X(6, 0), 2) * (cos_theta_t - cos_theta),
        //          0, 0, 1,                                       0, 0, 0,                                             0,
        //          0, 0, 0,                                       1, 0, 0,                                             0,
        //          0, 0, 0,                                       0, 1, 0,                                             0,
        //          0, 0, 0,                                       0, 0, 1,                                             m_dt,
        //          0, 0, 0,                                       0, 0, 0,                                             1;
        // }
        return F; 
    }

    Eigen::MatrixXd BalanceObserver::getTransformNoise(const Eigen::MatrixXd &X)
    {
        Eigen::Matrix<double, 7, 7> Q;
        double theta = X(5, 0);
        double v = X(2, 0);
        double dd = m_process_noise[0];
        double da = m_process_noise[1];
        double dz = m_process_noise[2];
        double dr = m_process_noise[3];
        double t5 = pow(m_dt, 5) / 5;
        double t4 = pow(m_dt, 4) / 4;
        double t3 = pow(m_dt, 3) / 3;
        double t2 = pow(m_dt, 2) / 2;
        double cos2 = pow(cos(theta), 2);
        double sin2 = pow(sin(theta), 2);
        double sincos = sin(theta) * cos(theta);
        Q << t5 * da * pow(v, 2) * sin2 / 4 + t3 * dd * cos2,         sin(theta) * t3 * dd - pow(v, 2) * sincos * t5 * da / 4, t2 * dd * cos(theta), 0,        0,        -t4 * da * v * sin(theta) / 2,-t3 * da * v * sin(theta) / 2,
             sin(theta) * t3 * dd - pow(v, 2) * sincos * t5 * da / 4, t3 * dd * sin2 + t5 * da * pow(v, 2) * cos2 / 4,         t2 * dd * sin(theta), 0,        0,         t4 * da * v * cos(theta) / 2, t3 * da * v * cos(theta) / 2,
             t2 * dd * cos(theta),                                    t2 * dd * sin(theta),                                    m_dt * dd,            0,        0,         0,                            0,
             0,                                                       0,                                                       0,                    m_dt *dz, 0,         0,                            0,
             0,                                                       0,                                                       0,                    0,        m_dt * dr, 0,                            0,
            -t4 * da * v * sin(theta) / 2,                            t4 * da * v * cos(theta) / 2,                            0,                    0,        0,         t3 * da,                      t2 * da,
            -t3 * da * v * sin(theta) / 2,                            t3 * da * v * cos(theta) / 2,                            0,                    0,        0,         t2 * da,                      m_dt * da;
        return Q;
    }

    Eigen::MatrixXd BalanceObserver::getPredictiveMeasurement(const Eigen::MatrixXd &X, int i)
    {
        Eigen::Matrix<double, 4, 1> h;
        h << X(0, 0) + X(4, 0) * cos(X(5, 0) + i * PI),
             X(1, 0) + X(4, 0) * sin(X(5, 0) + i * PI),
             X(3, 0),
             X(5, 0) + i * PI;
        return h;
    }

    Eigen::MatrixXd BalanceObserver::getMeasurementPD(const Eigen::MatrixXd &X, int i)
    {
        Eigen::Matrix<double, 4, 7> H;
        H << 1, 0, 0, 0, cos(X(5, 0) + i * PI),-X(4, 0) * sin(X(5, 0) + i * PI), 0,
             0, 1, 0, 0, sin(X(5, 0) + i * PI), X(4, 0) * cos(X(5, 0) + i * PI), 0,
             0, 0, 0, 1, 0,                     0,                               0,
             0, 0, 0, 0, 0,                     1,                               0;
        return H;
    }

    Eigen::MatrixXd BalanceObserver::getMeasureNoisePD(const Eigen::MatrixXd &X, int i)
    {
        Eigen::Matrix<double, 4, 1> h = getPredictiveMeasurement(X, i);
        double psi = atan2(h(1, 0), h(0, 0));
        double phi = atan2(h(2, 0), sqrt(pow(h(0, 0), 2) + pow(h(1, 0), 2)));
        double d = sqrt(pow(h(0, 0), 2) + pow(h(1, 0), 2) + pow(h(2, 0), 2));
        Eigen::Matrix<double, 4, 4> V;
        V << -d * cos(phi) * sin(psi), -d * sin(phi) * cos(psi), cos(phi) * cos(psi), 0,
              d * cos(phi) * cos(psi), -d * sin(phi) * sin(psi), cos(phi) * sin(psi), 0,
              0,                        d * cos(phi),            sin(phi),            0,
              0,                        0,                       0,                   1;
        return V;
    }


    OutpostObserver::OutpostObserver()
    :   ModelObserver()
    {
        setParam(AIM_CFG);

        reset();

        m_F << 1, 0, 0, 0, 0,
               0, 1, 0, 0, 0,
               0, 0, 1, 0, 0,
               0, 0, 0, 1, m_dt,
               0, 0, 0, 0, 1;

        double dd = m_process_noise[0];
        double da = m_process_noise[1];
        double dz = m_process_noise[2];
        double t4 = pow(m_dt, 3) / 3;
        double t3 = pow(m_dt, 2) / 2;
        double t2 = pow(m_dt, 1);
        m_Q << dd,      0,       0, 0,       0,
               0,       dd,      0, 0,       0,
               0,       0,       dz,0,       0,
               0,       0,       0, t4 * da, t3 * da,
               0,       0,       0, t3 * da, t2 * da;

        Eigen::VectorXd measurement_noise4(4);
        measurement_noise4 << m_measure_noise[0],
                              m_measure_noise[0],
                              m_measure_noise[1],
                              m_measure_noise[2];
        m_R = measurement_noise4.asDiagonal();
        Eigen::VectorXd measurement_noise8(8);
    }
    OutpostObserver::~OutpostObserver()
    {}

    void OutpostObserver::setParam(const std::string &file_path)
    {
        cv::FileStorage fs(file_path, cv::FileStorage::READ);
        fs["ModelObserver"]["debug"]    >> m_debug;
        fs["ModelObserver"]["dt"]       >> m_dt;

        fs["ModelObserver"]["Outpost"]["init_radius"]                          >> m_init_radius;
        fs["ModelObserver"]["Outpost"]["gain"]                                 >> m_gain;

        fs["ModelObserver"]["Outpost"]["process_noise"]["displace_high_diff"]  >> m_process_noise[0];
        fs["ModelObserver"]["Outpost"]["process_noise"]["anglar_high_diff"]    >> m_process_noise[1];
        fs["ModelObserver"]["Outpost"]["process_noise"]["height"]              >> m_process_noise[2];

        fs["ModelObserver"]["Outpost"]["measure_noise"]["pose"]                >> m_measure_noise[0];
        fs["ModelObserver"]["Outpost"]["measure_noise"]["distance"]            >> m_measure_noise[1];
        fs["ModelObserver"]["Outpost"]["measure_noise"]["angle"]               >> m_measure_noise[2];
        fs.release();

        m_X.resize(5, 1);
        m_X_update.resize(5, 1);
        m_P.resize(5, 5);
        m_F.resize(5, 5);
        m_Q.resize(5, 5);
        m_R.resize(4, 4);
    }

    void OutpostObserver::reset()
    {
        m_init = false;

        m_P = Eigen::Matrix<double, 5, 5>::Identity() * 1e-5;
    }

    std::pair<Eigen::MatrixXd, Eigen::MatrixXd> OutpostObserver::predict(const Armors &armors)
    {
        if (!m_init)
        {
            for (auto armor : armors)
            {
                // 取排序后第一块非白色装甲板计算初始化状态
                // 如果进入该初始化部分，前置逻辑已保证序列中一定有一个非白色装甲板
                if (armor.m_color != _COLOR::_WHITE)
                {
                    m_X << armor.m_position.x - m_init_radius * cos(armor.m_yaw_angle),
                           armor.m_position.y - m_init_radius * sin(armor.m_yaw_angle),
                           armor.m_position.z,
                           armor.m_yaw_angle,
                           0;
                    m_init = true;
                    break;
                }
            }
        }

        // 如果未识别到装甲板且转速较高时，以实际转速作为状态量
        if (armors.size() == 0 && abs(m_X(4, 0)) > 0.8)
        {
            double palstance = m_X(4, 0);

            std::cout << "pal: " << palstance << std::endl;

            // 根据变换后角速度判断转速
            if(abs(palstance) < 1.65)
                palstance = (palstance / abs(palstance)) * 1.25;
            else if (abs(palstance) > 2.1)
                palstance = (palstance / abs(palstance)) * 2.5;
            else
                palstance = m_X(4, 0);

            std::cout << "pal_: " << palstance << std::endl;

            m_X(4, 0) = palstance;
        }

        // 预测
        Eigen::MatrixXd X_ = m_F * m_X;
        Eigen::MatrixXd P_ = m_F * m_P * m_F.transpose() + m_Q;
        if (m_debug)
        {
            std::cout << "[OutpostObserver] X_: " << std::endl << X_ << std::endl;
            std::cout << "[OutpostObserver] P_: " << std::endl << P_ << std::endl;
        }

        return std::make_pair(X_, P_);
    }

    std::shared_ptr<KinematicModel> OutpostObserver::update(const Armors &armors, const Eigen::MatrixXd &X_, const Eigen::MatrixXd &P_, std::map<int, int> &match)
    {
        // 观测
        Eigen::MatrixXd Z;      // 实际观测量
        Eigen::MatrixXd h;      // 先验观测量
        Eigen::MatrixXd H;      // 观测方程偏导矩阵
        Eigen::MatrixXd V;      // 观测噪声偏导矩阵
        Eigen::MatrixXd K;      // 置信度权重矩阵
        Eigen::MatrixXd R;      // 时变观测噪声矩阵

        if (match.size() == 1)
        {
            int num = match.begin()->first;
            Z.resize(4, 1);
            Z << armors[num].m_position.x, armors[num].m_position.y, armors[num].m_position.z, armors[num].m_yaw_angle;
            h.resize(4, 1);
            h << getPredictiveMeasurement(X_, match[num]);
            H.resize(4, 5);
            H << getMeasurementPD(X_, match[num]);
            V.resize(4, 4);
            V << getMeasureNoisePD(X_, match[num]);
            R = m_R;
            R(2, 2) *= m_gain * abs(Z(3, 0));

            K = P_ * H.transpose() * ((H * P_ * H.transpose() + V * R * V.transpose())).inverse();
        }
        else
        {
            // 若匹配数目不为1，则认为丢识别或者误判，直接返回先验状态
            m_X = X_;
            m_P = P_;

            return std::make_shared<OutpostModel>(m_X);
        }

        // if (m_debug)
        // {
        //     std::cout << "[OutpostObserver] Z: " << std::endl << Z << std::endl;
        //     std::cout << "[OutpostObserver] h: " << std::endl << h << std::endl;
        //     std::cout << "[OutpostObserver] H: " << std::endl << H << std::endl;
        // }

        // 更新
        Eigen::MatrixXd tmp = Z - h;
        tmp(3, 0) = _std_radian(tmp(3, 0));
        m_X_update = K * tmp;
        if (m_debug)
        {
            std::cout << "[OutpostObserver] X_update: " << std::endl << m_X_update << std::endl;
        }

        m_X = X_ + m_X_update;
        m_P = (Eigen::Matrix<double, 5, 5>::Identity() - K * H) * P_;

        if (m_debug)
        {
            std::cout << "[OutpostObserver] X:" << std::endl << m_X << std::endl;
            std::cout << "[OutpostObserver] P:" << std::endl << m_P << std::endl;
        }

        return std::make_shared<OutpostModel>(m_X);
    }

    bool OutpostObserver::stable()
    {
        return true;
    }

    Eigen::MatrixXd OutpostObserver::getPredictiveMeasurement(const Eigen::MatrixXd &X, int i)
    {
        Eigen::Matrix<double, 4, 1> h;
        h << X(0, 0) + 0.25685 * cos(X(3, 0) + 2 * i * PI / 3),
             X(1, 0) + 0.25685 * sin(X(3, 0) + 2 * i * PI / 3),
             X(2, 0),
             X(3, 0) + 2 * i * PI / 3;
        return h;
    }

    Eigen::MatrixXd OutpostObserver::getMeasurementPD(const Eigen::MatrixXd &X, int i)
    {
        Eigen::Matrix<double, 4, 5> H;
        H << 1, 0, 0, -0.25685 * sin(X(3, 0) + 2 * i * PI / 3), 0,
             0, 1, 0,  0.25685 * cos(X(3, 0) + 2 * i * PI / 3), 0,
             0, 0, 1,                                        0, 0,
             0, 0, 0,                                        1, 0;
        return H;
    }

    Eigen::MatrixXd OutpostObserver::getMeasureNoisePD(const Eigen::MatrixXd &X, int i)
    {
        Eigen::Matrix<double, 4, 1> h = getPredictiveMeasurement(X, i);
        double psi = atan2(h(1, 0), h(0, 0));
        double phi = atan2(h(2, 0), sqrt(pow(h(0, 0), 2) + pow(h(1, 0), 2)));
        double d = sqrt(pow(h(0, 0), 2) + pow(h(1, 0), 2) + pow(h(2, 0), 2));
        Eigen::Matrix<double, 4, 4> V;
        V << -d * cos(phi) * sin(psi), -d * sin(phi) * cos(psi), cos(phi) * cos(psi), 0,
              d * cos(phi) * cos(psi), -d * sin(phi) * sin(psi), cos(phi) * sin(psi), 0,
              0,                        d * cos(phi),            sin(phi),            0,
              0,                        0,                       0,                   1;
        return V;
    }

}   // wmj
