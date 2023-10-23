#include "../include/Aimer.hpp"

namespace wmj
{
    Aimer::Aimer()
    {
        setParam(AIM_CFG);
#ifndef PARTIAL_COMPILE
        setCameraParam();
        m_roi_params = std::make_shared<SelectorParam>();
#endif
        m_MPC = std::make_shared<MPC>();

        reset();

        // m_match.debug = m_debug;
    }
    Aimer::~Aimer()
    {}

    void Aimer::setParam(const std::string &file_path)
    {
        cv::FileStorage fs(file_path, cv::FileStorage::READ);
        fs["Aimer"]["debug"]         >> m_debug;
        fs["Aimer"]["ekf_on"]        >> m_ekf_on;
        fs["Aimer"]["predict_on"]    >> m_predict_on;
        fs["Aimer"]["track_center"]  >> m_track_center;

        fs["Aimer"]["time_off"]                          >> m_time_off;
        fs["Aimer"]["switch_threshold"]                  >> m_switch_threshold;
        fs["Aimer"]["init_pose_tolerance"]               >> m_init_pose_tolerance;
        fs["Aimer"]["rubbish_data_tolerance"]            >> m_rubbish_data_tolerance;
        fs["Aimer"]["force_aim_palstance_threshold"]     >> m_force_aim_palstance_threshold;
        fs["Aimer"]["aim_angle_tolerance"]               >> m_aim_angle_tolerance;
        fs["Aimer"]["aim_pose_tolerance"]                >> m_aim_pose_tolerance;
        fs["Aimer"]["score_tolerance"]                   >> m_score_tolerance;
        fs["Aimer"]["all_white_tolerance_stop_shoot"]    >> m_all_white_tolerance_stop_shoot;
        fs["Aimer"]["all_white_tolerance_reset"]         >> m_all_white_tolerance_reset;
        fs["Aimer"]["shoot_interval"]                    >> m_shoot_interval;

        fs["Aimer"]["TRACK_CENTER"]["aim_center_palstance_threshold"]    >> m_aim_center_palstance_threshold;
        fs["Aimer"]["TRACK_CENTER"]["switch_trackmode_threshold"]        >> m_switch_trackmode_threshold;
        fs["Aimer"]["TRACK_CENTER"]["aim_center_angle_tolerance"]        >> m_aim_center_angle_tolerance;
#ifndef PARTIAL_COMPILE
        fs["Aimer"]["ROI"]["min_height"]                 >> m_min_roi_height;
        fs["Aimer"]["ROI"]["roi_height_zoom_rate"]       >> m_roi_height_zoom_rate;
        fs["Aimer"]["ROI"]["roi_width_zoom_rate"]        >> m_roi_width_zoom_rate;
#endif
        fs.release();
    }

    void Aimer::reset()
    {
        m_tracking = false;
        m_enable_shoot = false;
        m_tracked_ID = -1;
        m_type_init_cnt = 0;
        m_track_lost_cnt = 0;
        m_all_white_cnt = 0;
        m_next_shoot_time = now();
#ifndef PARTIAL_COMPILE
        m_deep_roi_state = ROI_BIG;
#ifdef USE_DEEP
        m_return_roi_left = m_roi_params->m_deep_default_roi;
        m_return_roi_right = m_roi_params->m_deep_default_roi;
#else
        m_return_roi_right = m_roi_params->m_camera_resolution;
        m_return_roi_left = m_roi_params->m_camera_resolution;
#endif
#endif
        m_model_type = KinematicModel::UNKNOWN;
        m_status = std::make_shared<StandardModel>();   // 只为初始化index，具体设为何种模型并无实际影响

        if (m_model_observer)
        {
            m_model_observer->reset();
        }

        if (m_debug)
            std::cout << _warning("[Aimer] Reset") << std::endl;
    }

    GimbalPose Aimer::getTargetPose(const Armors &armors, const GimbalPose &cur_pose, double bullet_speed, int control_mode)
    {
        m_cur_pose = cur_pose;

        // 初始化锁ID
        if (m_tracked_ID == -1)
        {
            // 建立一个初始化序列，根据瞄准所需位姿和当前位姿的差值对所有识别到的装甲板正序排序
            Armors init_armors = armors;
            sort(init_armors.begin(), init_armors.end(),
                [&](Armor &a, Armor &b)
                {
                    return (m_angle_solver.getAngle(a.m_position, m_cur_pose, bullet_speed) - m_cur_pose).norm() < 
                           (m_angle_solver.getAngle(b.m_position, m_cur_pose, bullet_speed) - m_cur_pose).norm();
                }
            );
            // 在最接近当前瞄准的装甲板排序中，选择最优的非白色装甲板ID作为目标，且该目标与当前瞄准位置偏差在容忍范围内
            // 即防止直接瞄准死车，或因为近处目标丢识别而甩飞；但是当与死车同ID的活车出现时，还是可以锁上该ID
            for (auto armor : init_armors)
            {
                if (armor.m_color != _COLOR::_WHITE)
                {
                    if ((m_angle_solver.getAngle(armor.m_position, m_cur_pose, bullet_speed) - m_cur_pose).norm() <
                        D2R(m_init_pose_tolerance))
                    {
                        m_tracked_ID = armor.m_id;
                        break;
                    }
                }
            }
        }
        // 如果ID未初始化则直接返回当前位姿
        if (m_tracked_ID == -1)
        {
            m_tracking = false;
            m_enable_shoot = false;
            return m_cur_pose;
        }
        if (m_debug)
            std::cout << "[Aimer] id: " << m_tracked_ID << std::endl;

        // 建立目标装甲板序列
        Armors target_armor_seq;
        bool all_white = true;
        for (auto armor : armors)
        {
            // 将与目标ID相同、目标大小相同且角度在可接受范围内的装甲板填入序列中
            // 目前仍是相机系
            if (armor.m_id == m_tracked_ID && abs(R2D(armor.m_yaw_angle)) < m_rubbish_data_tolerance
                && ((armor.m_armor_type == ARMOR_LARGE && m_model_type == KinematicModel::BALANCE)
                || (armor.m_armor_type == ARMOR_SMALL && m_model_type != KinematicModel::BALANCE)
                || (armor.m_armor_type == ARMOR_LARGE && m_tracked_ID == 1 && KinematicModel::STANDARD)
                || (KinematicModel::UNKNOWN)))
            {
                target_armor_seq.emplace_back(armor);
                if (armor.m_color != _COLOR::_WHITE)
                {
                    all_white = false;
                }
            }
        }

        // 判断当前序列是否全部为白色装甲板
        if (all_white)
        {
            m_all_white_cnt++;
            std::cout << _red("[Aimer] all white !") << std::endl;
        }
        else
        {
            m_all_white_cnt = 0;
        }

        // 若连续多帧没有识别到目标ID下亮着的装甲板，则认为目标丢失，自动重置
        if (m_all_white_cnt > m_all_white_tolerance_reset)
        {
            reset();
            return m_cur_pose;
        }

        if (m_debug)
            std::cout << "[Aimer] model type: " << m_model_type << std::endl;
        // 判断模型类型
        if (m_type_init_cnt < 3)
        {
            if (target_armor_seq.size() > 0)
            {
                Armor armor = target_armor_seq[0];
                // 英雄、非平衡步兵与哨兵
                if ((armor.m_armor_type == ARMOR_LARGE && armor.m_id == 1) ||
                    (armor.m_armor_type == ARMOR_SMALL && armor.m_id >= 2 && armor.m_id <= 7))
                {
                    if (m_model_type != KinematicModel::STANDARD)
                        m_type_init_cnt = 0;
                    m_model_type = KinematicModel::STANDARD;
                    m_type_init_cnt++;
                }
                // 平衡步兵
                else if (armor.m_armor_type == ARMOR_LARGE && armor.m_id >= 3 && armor.m_id <= 5)
                {
                    if (m_model_type != KinematicModel::BALANCE)
                        m_type_init_cnt = 0;
                    m_model_type = KinematicModel::BALANCE;
                    m_type_init_cnt++;
                }
                // 前哨站
                else if (armor.m_id == 11)
                {
                    if (m_model_type != KinematicModel::OUTPOST)
                        m_type_init_cnt = 0;
                    m_model_type = KinematicModel::OUTPOST;
                    m_type_init_cnt++;
                }
            }

            if (m_type_init_cnt >= 3)
            {
                setModelType(m_model_type);
            }
            else
            {
                // 未完成类别判断，返回当前位姿
                m_enable_shoot = false;
                return m_cur_pose;
            }
        }

        m_tracking = true;

        // 异常数据检查
        errorHandling(target_armor_seq);

        // 根据瞄准所需位姿和当前位姿的差值对目标ID的装甲板正序排序
        sort(target_armor_seq.begin(), target_armor_seq.end(),
            [&](Armor &a, Armor &b)
            {
                return (m_angle_solver.getAngle(a.m_position, m_cur_pose, bullet_speed) - m_cur_pose).norm() < 
                       (m_angle_solver.getAngle(b.m_position, m_cur_pose, bullet_speed) - m_cur_pose).norm();
            }
        );

        // 将目标装甲板序列的坐标和角度转为绝对系
        for (auto &armor : target_armor_seq)
        {
            armor.m_position = m_angle_solver.cam2abs(armor.m_position, m_cur_pose);
            armor.m_yaw_angle = _std_radian(armor.m_yaw_angle + m_cur_pose.yaw + PI);
        }

        // 目标装甲板
        Armor abs_facing_armor;
        Armor abs_target_armor;
        int last_index = m_status->index;
        if (m_ekf_on)
        {
            // 使用EKF进行建模
            m_status = resolve(target_armor_seq);

            double hit_time = 0;
            double center_hit_time = 0;
            if (m_predict_on)
            {
                /**
                 * 严格意义上来说，如果要准确预测击中时刻的装甲板位置的话，需要解一个非线性方程。此处采用一种近似的解法
                 * 根据当前最近装甲板距离计算击中时间，用于预测目标装甲板出现的位置
                 * 事实上相当于一步牛顿迭代法，或者说一阶的线性化
                 */
                center_hit_time = getDistance(m_status->getFacingArmor(0).m_position) / bullet_speed + m_time_off;
                hit_time = getDistance(m_status->getClosestArmor(0, 0).m_position) / bullet_speed + m_time_off;
            }
            abs_facing_armor = m_status->getFacingArmor(center_hit_time);
            abs_target_armor = m_status->getClosestArmor(hit_time, m_switch_threshold);
        }
        else
        {
            // 否则直接返回瞄准最佳的装甲板
            abs_facing_armor = target_armor_seq[0];
            abs_target_armor = target_armor_seq[0];
        }

#ifndef PARTIAL_COMPILE
        // 计算ROI
        setROI(m_status, target_armor_seq);
#endif

        // 判断是否锁中心
        if (m_track_center && ((abs(m_status->palstance) - m_switch_trackmode_threshold) > m_aim_center_palstance_threshold))
        {
            m_center_tracked = true;
        }
        else
        {
            m_center_tracked = false;
        }

        /**
         * 判断自动击发，条件如下：
         * 1. 目标旋转速度小于一定阈值，或目标装甲板相对偏角不超过一定范围
         * 2. EKF先验稳定
         * 3. 没有长时间丢识别
         * 4. 当前云台跟随稳定，即当前位姿和目标位姿相近
         */
        cv::Point2d vector_c = m_status->center;
        cv::Point2d vector_a = cv::Point2d(m_status->center.x - abs_target_armor.m_position.x, m_status->center.y - abs_target_armor.m_position.y);
        double abs_angle = R2D(acos((vector_c.x * vector_a.x + vector_c.y * vector_a.y) / (getDistance(vector_c) * getDistance(vector_a))));
        if ((abs(m_status->palstance) < m_force_aim_palstance_threshold || abs_angle < m_aim_angle_tolerance) &&
            m_model_observer->stable() &&
            m_all_white_cnt < m_all_white_tolerance_stop_shoot &&
            (m_target_pose - m_cur_pose).norm() < m_aim_pose_tolerance
            )
        {
            m_enable_shoot = true;
            if (m_center_tracked && !(abs_angle < m_aim_center_angle_tolerance))
            {
                m_enable_shoot = false;
            }
        }
        else
        {
            m_enable_shoot = false;
        }

        if (control_mode == 0)
        {
            // 计算目标位姿, 只有转速大于一定值才跟随圆心
            if (m_center_tracked)
                m_target_pose = m_angle_solver.getAngle(m_angle_solver.abs2cam(abs_facing_armor.m_position, m_cur_pose), m_cur_pose, bullet_speed);
            else
                m_target_pose = m_angle_solver.getAngle(m_angle_solver.abs2cam(abs_target_armor.m_position, m_cur_pose), m_cur_pose, bullet_speed);
        }
        else
        {
            m_target_pose = m_MPC->getGimbalSpeed(m_status, cur_pose, bullet_speed);
        }

        return m_target_pose;
    }

    std::shared_ptr<KinematicModel> Aimer::resolve(const Armors &armors)
    {
        std::pair<Eigen::MatrixXd, Eigen::MatrixXd> prior = m_model_observer->predict(armors);
        std::shared_ptr<KinematicModel> prior_status;
        if (m_model_type == KinematicModel::STANDARD)
        {
            prior_status = std::make_shared<StandardModel>(prior.first);
        }
        else if (m_model_type == KinematicModel::BALANCE)
        {
            prior_status = std::make_shared<BalanceModel>(prior.first);
        }
        else if (m_model_type == KinematicModel::OUTPOST)
        {
            prior_status = std::make_shared<OutpostModel>(prior.first);
        }

        if (m_debug)
            prior_status->print("prior");

        Eigen::MatrixXd score = getScoreMat(armors, prior_status->getArmors(0));
        if (m_debug)
            std::cout << "score: " << std::endl << score << std::endl;

        // std::map<int, int> armor_match = match(score);
        std::map<int, int> armor_match = m_match.getMatch(score, m_score_tolerance, prior_status->number);

        std::shared_ptr<KinematicModel> posterior_status = m_model_observer->update(armors, prior.first, prior.second, armor_match);
        if (m_debug)
            posterior_status->print("posterior");

        return posterior_status;
    }

    bool Aimer::isReady()
    {
        return m_tracking;
    }

    bool Aimer::shootable()
    {
        if (m_enable_shoot && now() > m_next_shoot_time)
        {
            m_next_shoot_time = now() + m_shoot_interval;
            return true;
        }
        return false;
    }

    void Aimer::errorHandling(Armors &armors)
    {
        for (auto it = armors.begin(); it != armors.end();)
        {
            if (isinf(it->m_position.x) || isnan(it->m_position.x) ||
                isinf(it->m_position.y) || isnan(it->m_position.y) ||
                isinf(it->m_position.z) || isnan(it->m_position.z) ||
                isinf(it->m_yaw_angle)  || isnan(it->m_yaw_angle)
                )
            {
                std::cout << _lightred("[Aimer] Error data: inf or nan") << std::endl;
                it = armors.erase(it);
            }
            else
            {
                ++it;
            }
        }
    }

    void Aimer::setModelType(KinematicModel::Type type)
    {
        m_model_type = type;
        // 大装甲板击发位姿增大
        if (m_tracked_ID == 1 || type == KinematicModel::BALANCE)
            m_aim_pose_tolerance *= 1.5;

        // 平衡丢识别允许次数增加
        if (type == KinematicModel::BALANCE || type == KinematicModel::OUTPOST)
            m_all_white_tolerance_stop_shoot *= 5;

        if (type == KinematicModel::STANDARD)
        {
            m_status = std::make_shared<StandardModel>();
            m_model_observer = std::make_shared<StandardObserver>();
            std::cout << _lightred("[Aimer] Model type has been set to STANDARD") << std::endl;
        }
        else if (type == KinematicModel::BALANCE)
        {
            m_status = std::make_shared<BalanceModel>();
            m_model_observer = std::make_shared<BalanceObserver>();
            std::cout << _lightred("[Aimer] Model type has been set to BALANCE") << std::endl;
        }
        else if (type == KinematicModel::OUTPOST)
        {
            m_status = std::make_shared<OutpostModel>();
            m_model_observer = std::make_shared<OutpostObserver>();
            std::cout << _lightred("[Aimer] Model type has been set to OUTPOST") << std::endl;
        }
    }

    Eigen::MatrixXd Aimer::getScoreMat(const Armors &detect_armors, const Armors &standard_armors)
    {
        int m = detect_armors.size();
        int n = standard_armors.size();
        // 计算两组装甲板之间的坐标差和角度差两个负向指标
        Eigen::Matrix<double, Eigen::Dynamic, 2> negative_score;
        negative_score.resize(m * n, 2);
        for (int i = 0; i < m; i++)
        {
            for (int j = 0; j < n; j++)
            {
                negative_score(i * n + j, 0) = getDistance(detect_armors[i].m_position - standard_armors[j].m_position);
                negative_score(i * n + j, 1) = abs(_std_radian(detect_armors[i].m_yaw_angle - standard_armors[j].m_yaw_angle));
            }
        }

        // 数据标准化
        Eigen::Matrix<double, Eigen::Dynamic, 2> regular_score;
        regular_score.resize(m * n, 2);
        for (int i = 0; i < regular_score.rows(); i++)
        {
            regular_score(i, 0) = (negative_score.col(0).maxCoeff() - negative_score(i, 0)) / (negative_score.col(0).maxCoeff() - negative_score.col(0).minCoeff());
            regular_score(i, 1) = (negative_score.col(1).maxCoeff() - negative_score(i, 1)) / (negative_score.col(1).maxCoeff() - negative_score.col(1).minCoeff());
        }

        // 计算样本值占指标的比重
        Eigen::Matrix<double, Eigen::Dynamic, 2> score_weight;
        score_weight.resize(m * n, 2);
        Eigen::VectorXd col_sum = regular_score.colwise().sum();
        for (int i = 0; i < score_weight.rows(); i++)
        {
            score_weight(i, 0) = regular_score(i, 0) / col_sum(0);
            score_weight(i, 1) = regular_score(i, 1) / col_sum(1);
        }

        // 计算每项指标的熵值
        Eigen::Vector2d entropy = Eigen::Vector2d::Zero();
        for (int i = 0; i < score_weight.rows(); i++)
        {
            if (score_weight(i, 0) != 0)
                entropy(0) -= score_weight(i, 0) * log(score_weight(i, 0));
            if (score_weight(i, 1) != 0)
                entropy(1) -= score_weight(i, 1) * log(score_weight(i, 1));
        }
        entropy /= log(score_weight.rows());

        // 计算权重
        Eigen::Vector2d weight = (Eigen::Vector2d::Ones() - entropy) / (2 - entropy.sum());

        // 计算匹配得分矩阵
        Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> score;
        score.resize(m, n);
        for (int i = 0; i < m; i++)
        {
            for (int j = 0; j < n; j++)
            {
                if (i < detect_armors.size() && j < standard_armors.size())
                {
                    score(i, j) = negative_score.row(i * standard_armors.size() + j) * weight;
                }
            }
        }

        return score;
    }

    std::map<int, int> Aimer::match(const Eigen::MatrixXd &score)
    {
        std::map<int, int> match;
        for (int i = 0; i < score.rows(); i++)
        {
            int index;
            score.row(i).minCoeff(&index);
            match[i] = index;
            // std::cout << "(" << i << ", " << match[i] << ") ";
        }
        // std::cout << std::endl;
        return match;
    }


    Aimer::Match::Match()
    {}
    Aimer::Match::~Match()
    {}

    std::map<int, int> Aimer::Match::getMatch(Eigen::MatrixXd matrix, double score_max, int m)
    {
        /**
         * @brief 初始化各个变量
         *
         */
        /// 初始化最终结果
        this->row_col.clear();
        for (int i = 0; i < matrix.rows(); i++)
        {
            // 值为-1表示不选择该行中的任何数
            this->row_col[i] = -1;
        }
        /// 初始化min
        for (int i = 0; i < matrix.rows() && i < m; i++)
        {
            this->min += matrix(i, i);
            this->row_col[i] = i;
        }
        /// 初始化行数列
        this->row.clear();
        for (int i = 0; i < matrix.rows(); i++)
        {
            // 有多少行，行数列就有多少个数
            this->row.emplace_back(i);
        }
        /// 初始化列数列，n块装甲板，因此列为n
        this->col.clear();
        for (int i = 0; i < m; i++)
        {
            this->col.emplace_back(i);
        }

        this->tmp_v.clear();
        this->result.clear();
        this->nAfour.clear();
        this->fourAfour.clear();

        /**
         * @brief 用A(n,m)求出可能选择的所有行的组合
         * 因为C(n,m) * A(m,m) = A(n,m)，所以先计算C(n,m)，再计算A(m,m)，最后将结果存到result中
         *
         */
        /// 计算C(n,m)
        if (matrix.rows() <= m)
            this->getCombinationsNumbers(this->row, this->tmp_v, this->result, 0, matrix.rows());
        else
            this->getCombinationsNumbers(this->row, this->tmp_v, this->result, 0, m);
        // 输出C(n,m)计算结果
        if (this->debug)
        {
            // 红色输出
            std::cout << _red("C(n,m)生成结果") << std::endl;
            for (auto it = this->result.begin(); it != this->result.end(); it++)
            {
                for (std::vector<int>::iterator it2 = it->begin(); it2 != it->end(); it2++)
                {
                    std::cout << *it2 << " ";
                }
                std::cout << std::endl;
            }
            std::cout << "-------------------------------" << std::endl;
        }
        /// 使用上一步的结果计算A(m,m)，最终得到A(n,m)
        for (auto it = this->result.begin(); it != this->result.end(); it++)
        {
            do
            {
                this->nAfour.emplace_back(*it);
            } while (next_permutation(it->begin(), it->end())); // stl自带全排列函数
        }
        // 输出A(n,m)计算结果
        if (this->debug)
        {
            // 绿色输出
            std::cout << _green("A(n,m)生成结果") << std::endl;
            for (auto it = this->nAfour.begin(); it != this->nAfour.end(); it++)
            {
                for (std::vector<int>::iterator it2 = it->begin(); it2 != it->end(); it2++)
                {
                    std::cout << *it2 << " ";
                }
                std::cout << std::endl;
            }
            std::cout << "-------------------------------" << std::endl;
        }

        /**
         * @brief 用A(m,m)求出可能选择的所有列的组合
         *
         */
        /// 计算A(m,m)
        do
        {
            fourAfour.push_back(col);
        } while (next_permutation(col.begin(), col.end())); // stl自带全排列函数
        // 输出A(m,m)计算结果
        if (this->debug)
        {
            // 蓝色输出
            std::cout << _blue("A(m,m)生成结果") << std::endl;
            for (auto it = this->fourAfour.begin(); it != this->fourAfour.end(); it++)
            {
                for (std::vector<int>::iterator it2 = it->begin(); it2 != it->end(); it2++)
                {
                    std::cout << *it2 << " ";
                }
                std::cout << std::endl;
            }
            std::cout << "-------------------------------" << std::endl;
        }

        /**
         * @brief 用A(n,m)和A(m,m)求出可能选择的所有行列的组合的结果
         *
         */
        for (auto it = this->nAfour.begin(); it != this->nAfour.end(); it++)
        {
            for (auto it2 = this->fourAfour.begin(); it2 != this->fourAfour.end(); it2++)
            {
                // 对于每一种组合，都求一遍它们的值，与min比较，如果小于min，则更新min
                this->tmp = 0;
                for (int i = 0; i < it->size(); i++)
                    this->tmp += matrix((*it)[i], (*it2)[i]);
                if (this->tmp < this->min)
                {
                    this->min = this->tmp;
                    // 重新初始化map
                    this->row_col.clear();
                    // 每选择矩阵中的一个数时，就将该位置记录下来存到map中
                    for (int i = 0; i < it->size(); i++)
                    {
                        this->row_col[it->at(i)] = it2->at(i);
                    }
                }
            }
        }
        // 输出删除前的map
        if (this->debug)
        {
            // 紫色输出
            std::cout << _purple("删除-1前的map结果") << std::endl;
            for (auto it = this->row_col.begin(); it != this->row_col.end(); it++)
            {
                std::cout << it->first << " " << it->second << std::endl;
            }
            std::cout << "-------------------------------" << std::endl;
        }
        /// 删除-1的键值对
        for (auto it = this->row_col.begin(); it != this->row_col.end();)
        {
            if (it->second == -1 || (it->second != -1 && matrix(it->first, it->second) > score_max))
            {
                it = this->row_col.erase(it);
            }
            else
                ++it;
        }
        // 输出min的值和删除后的map
        if (this->debug)
        {
            // 白色输出
            std::cout << _white("min和删除-1后的map结果") << std::endl;
            std::cout << "min = " << min << std::endl;
            for (auto it = this->row_col.begin(); it != this->row_col.end(); ++it)
            {
                std::cout << it->first << " " << it->second << " " << matrix(it->first, it->second) << std::endl;
            }
            std::cout << "-------------------------------" << std::endl;
        }
        /// 输出矩阵
        if (this->debug)
        {
            std::cout << "matrix:" << std::endl;
            for (int i = 0; i < matrix.rows(); i++)
            {
                for (int j = 0; j < matrix.cols(); j++)
                {
                    std::cout << matrix(i, j) << " ";
                }
                std::cout << std::endl;
            }
            std::cout << std::endl;
        }

        return this->row_col;
    }

    void Aimer::Match::getCombinationsNumbers(std::vector<int> &input, std::vector<int> &tmp_v, std::vector<std::vector<int>> &result, int start, int k)
    {
        for (int i = start; i < input.size(); i++)
        {
            tmp_v.emplace_back(input[i]);
            if (tmp_v.size() == k)
            {
                result.emplace_back(tmp_v);
                tmp_v.pop_back();
                continue;
            }
            // 使用递归计算
            getCombinationsNumbers(input, tmp_v, result, i + 1, k);
            tmp_v.pop_back();
        }
    }
#ifndef PARTIAL_COMPILE
/************ ROI ************/
    void Aimer::setCameraParam()
    {
        Monocular mono;
#ifdef USE_BINO
        m_camera_mat = mono.m_CameraMat_Left;
#else
        m_camera_mat = mono.m_CameraMat;
#endif
        return;
    }

    void Aimer::setROI(std::shared_ptr<KinematicModel> status, const Armors &armors)
    {
        cv::Rect2d roi;
        double m_roi_value = m_roi_params->m_roi_lost_mul[m_track_lost_cnt + 1];

#ifdef USE_DEEP
            // std::cout << "DEEP SELECT" << std::endl;
            if (armors.empty()) 
            {
                if (m_track_lost_cnt > m_roi_params->m_max_track_lost)
                {
                    // roi返回为中央roi
                    m_deep_roi_state = wmj::DeepROISizeState::ROI_BIG;
                    m_return_roi_left = m_roi_params->m_deep_default_roi;
                    m_return_roi_right = m_roi_params->m_deep_default_roi;
                    return;
                }
                else
                {
                    m_track_lost_cnt++;
                }

            }
            else
            {
                m_track_lost_cnt = 0;
            }

            Armors pre_armors = status->getArmors();
            std::vector<cv::Point2f> reprojection_points;
            for (auto &armor : armors)
            {
                reprojection_points.emplace_back(getReProjectPoint(m_angle_solver.abs2cam(armor.m_position, m_cur_pose)));
                for (int i = 0; i < 4; i++)
                {
                    reprojection_points.emplace_back(armor.m_vertices[i]);
                }
            }
            for (auto &armor : pre_armors)
            {
                reprojection_points.emplace_back(getReProjectPoint(m_angle_solver.abs2cam(armor.m_position, m_cur_pose)));
            }
            reprojection_points.emplace_back(getReProjectPoint(m_angle_solver.abs2cam(cv::Point3f(m_status->center.x, m_status->center.y, m_status->height[0]), m_cur_pose)));

            float max_x, min_x, max_y, min_y;
            max_x = min_x = reprojection_points[0].x;
            max_y = min_y = reprojection_points[0].y;
            for (auto &point : reprojection_points)
            {
                max_x = max_x > point.x ? max_x : point.x;
                min_x = min_x < point.x ? min_x : point.x;
                max_y = max_y > point.y ? max_y : point.y;
                min_y = min_y < point.y ? min_y : point.y;
            }
            float height,width;
            height = max_y - min_y;
            width = max_x - min_x;
            roi = cv::Rect2d(min_x - (1.3 - 1) * 0.5 * width, min_y - (1.3 - 1) * 0.5 * height, width * 1.3, height * 1.3);
            roi &= m_roi_params->m_camera_resolution;
            if (roi.height * 1.3 > m_roi_params->m_deep_roi_size.height || roi.width * 1.5 > m_roi_params->m_deep_roi_size.width)
            {
                m_deep_roi_state = wmj::DeepROISizeState::ROI_BIG;
                roi = m_roi_params->m_deep_default_roi; 
                roi.y = min_y + (height - roi.height) / 2.0;
                roi.y = std::max(std::min((float)roi.y, (float)(m_roi_params->m_camera_resolution.height - 1 - m_roi_params->m_deep_default_roi.height)), 1.f);
            }

            else if (height * 1.5 > m_roi_params->m_deep_roi_size.height || width * 1.9 > m_roi_params->m_deep_roi_size.width)
            {
                if (m_deep_roi_state == wmj::DeepROISizeState::ROI_SMALL)
                {
                    roi.y += (roi.height - m_roi_params->m_deep_roi_size.height) / 2.0;
                    roi.height = m_roi_params->m_deep_roi_size.height;
                    roi.x += (roi.width - m_roi_params->m_deep_roi_size.width) / 2.0;
                    roi.width = m_roi_params->m_deep_roi_size.width;
                    roi.x = std::max(std::min((float)roi.x, (float)(m_roi_params->m_camera_resolution.width - 1 - m_roi_params->m_deep_roi_size.width)), 1.f);
                    roi.y = std::max(std::min((float)roi.y, (float)(m_roi_params->m_camera_resolution.height - 1 - m_roi_params->m_deep_roi_size.height)), 1.f);
                }
                else
                {
                    roi = m_roi_params->m_deep_default_roi; 
                    roi.y = min_y + (height - roi.height) / 2.0;
                    roi.y = std::max(std::min((float)roi.y, (float)(m_roi_params->m_camera_resolution.height - 1 - m_roi_params->m_deep_default_roi.height)), 1.f);
                }
            }
            else
            {
                m_deep_roi_state = wmj::DeepROISizeState::ROI_SMALL;
                roi.y += (roi.height - m_roi_params->m_deep_roi_size.height) / 2.0;
                roi.height = m_roi_params->m_deep_roi_size.height;
                roi.x += (roi.width - m_roi_params->m_deep_roi_size.width) / 2.0;
                roi.width = m_roi_params->m_deep_roi_size.width;
                roi.x = std::max(std::min((float)roi.x, (float)(m_roi_params->m_camera_resolution.width - 1 - m_roi_params->m_deep_roi_size.width)), 1.f);
                roi.y = std::max(std::min((float)roi.y, (float)(m_roi_params->m_camera_resolution.height - 1 - m_roi_params->m_deep_roi_size.height)), 1.f);
            }
            if (roi.area() == 0)
            {
                roi = m_roi_params->m_deep_default_roi;
            }
            m_return_roi_right = roi;
            m_return_roi_left = roi;
            return;

#else
            if (armors.empty())
            {
                if (m_track_lost_cnt > m_roi_params->m_max_track_lost)
                {
                    // roi返回为中央roi
                    m_return_roi_left = m_roi_params->m_camera_resolution;
                    m_return_roi_right =  m_roi_params->m_camera_resolution;
                    return;
                }
                else
                {
                    m_track_lost_cnt++;
                }
            }
            else
            {
                m_track_lost_cnt = 0;
            }
            // 如果为哨兵模式则返回一个轨道狭长长矩形的roi区域
            if (m_tracked_ID == 7 && !armors.empty())
            {
                roi = armors[0].m_rect;
                roi.y -= roi.height * m_roi_params->m_sentry_roi_up_ratio * m_roi_value;
                roi.height *= 1.0 + m_roi_params->m_sentry_roi_up_ratio * m_roi_value + m_roi_params->m_sentry_roi_down_ratio * m_roi_value;
                roi.x -= roi.width * m_roi_params->m_sentry_roi_left_ratio * m_roi_value;
                roi.width *= 1.0 + m_roi_params->m_sentry_roi_left_ratio * m_roi_value + m_roi_params->m_sentry_roi_right_ratio * m_roi_value;
                // 防止越界 
                roi &= m_roi_params->m_camera_resolution;
            }
            // 如果为反陀螺和常规模式，则返回一个全车装甲板的矩形roi区域
            else
            {
                Armors pre_armors = status->getArmors();
                std::vector <cv::Point2f> reprojection_points;
                for (auto &armor : armors)
                {
                    reprojection_points.emplace_back(getReProjectPoint(m_angle_solver.abs2cam(armor.m_position, m_cur_pose)));
                }
                for (auto &armor : pre_armors)
                {
                    reprojection_points.emplace_back(getReProjectPoint(m_angle_solver.abs2cam(armor.m_position, m_cur_pose)));
                }
                reprojection_points.emplace_back(getReProjectPoint(m_angle_solver.abs2cam(cv::Point3f(status->center.x, status->center.y, status->height[0]),m_cur_pose)));

                float max_x, min_x, max_y, min_y;
                max_x = min_x = reprojection_points[0].x;
                max_y = min_y = reprojection_points[0].y;
                for(auto &point : reprojection_points)
                {
                    max_x = max_x > point.x ? max_x : point.x;
                    min_x = min_x < point.x ? min_x : point.x;
                    max_y = max_y > point.y ? max_y : point.y;
                    min_y = min_y < point.y ? min_y : point.y;
                }

                float height,width;
                height = max_y - min_y;
		        height = height > m_min_roi_height ? height : m_min_roi_height;
                width = max_x - min_x;
                roi = cv::Rect2d(
                    min_x - (m_roi_width_zoom_rate - 1) * 0.5 * width,
                    min_y - (m_roi_height_zoom_rate - 1) * 0.5 * height,
                    width * m_roi_width_zoom_rate,
                    height * m_roi_height_zoom_rate
                );
                roi &= m_roi_params->m_camera_resolution;
            }

            if (roi.area() == 0)
            {
                roi = m_roi_params->m_camera_resolution;
            }

            // armors为空 双目或左目
            if (armors.empty() || armors[0].m_detectedtype == 0 || armors[0].m_detectedtype == 2)
            {
                m_return_roi_left = roi;
                m_return_roi_right = m_return_roi_left;
                m_return_roi_right.x -= m_roi_params->m_aim_point_dis;

                m_return_roi_left  &= m_roi_params->m_camera_resolution;
                m_return_roi_right &= m_roi_params->m_camera_resolution;

            }
            // 右目
            else
            {
                m_return_roi_right = roi;
                m_return_roi_left = roi;
                m_return_roi_left.x += m_roi_params->m_aim_point_dis;

                m_return_roi_left  &= m_roi_params->m_camera_resolution;
                m_return_roi_right &= m_roi_params->m_camera_resolution;
            }
#endif
        return;
    }

    void Aimer::setDeepROISize(cv::Size2i deep_roi_size)
    {
	    m_roi_params->m_deep_roi_size = deep_roi_size;
	    double rate = (double)deep_roi_size.height / (double)deep_roi_size.width;
	    m_roi_params->m_deep_default_roi = cv::Rect2d(
            0,
            (m_roi_params->m_camera_resolution.height - m_roi_params->m_camera_resolution.width * rate) / 2,
            m_roi_params->m_camera_resolution.width,
            (m_roi_params->m_camera_resolution.width * rate)
        );
        m_return_roi_left = m_roi_params->m_deep_default_roi;
        m_return_roi_right = m_roi_params->m_deep_default_roi;
    }

    cv::Rect2d Aimer::getLeftROI()
    {
        return m_return_roi_left;
    }
    cv::Rect2d Aimer::getRightROI()
    {
        return m_return_roi_right;
    }

    void Aimer::drawReProjectPoint(cv::Mat &src)
    {
		Armors pre_armors = m_status->getArmors();
        std::vector<cv::Point2f> reprojection_points;
        for (auto &armor : pre_armors)
        {
            reprojection_points.emplace_back(getReProjectPoint(m_angle_solver.abs2cam(armor.m_position, m_cur_pose)));
        }
		cv::Point2f center = getReProjectPoint(m_angle_solver.abs2cam(cv::Point3f(m_status->center.x, m_status->center.y, m_status->height[0]), m_cur_pose));
		cv::circle(src, center, 8, cv::Scalar(255, 0, 255), -1);
		for (auto point : reprojection_points)
		{
		    cv::circle(src, point, 10, cv::Scalar(255, 255, 0), 3);
		}
        cv::rectangle(src, m_return_roi_right, cv::Scalar(0,0,255), 2);
        return;
    }

    cv::Point2f Aimer::getReProjectPoint(const cv::Point3f &point)
    {
        cv::Mat_<double> point_mat = (cv::Mat_<double>(3, 1) << point.y / -1000, point.z / -1000, point.x / 1000);
        cv::Mat reprojection_point = m_camera_mat * point_mat / (point.x / 1000);
        return cv::Point2f(reprojection_point.at<double>(0, 0), reprojection_point.at<double>(1, 0));
    }
/************ ROI ************/
#endif
}   // wmj
