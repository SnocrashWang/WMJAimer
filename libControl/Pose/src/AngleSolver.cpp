#include "../include/AngleSolver.hpp"
#include <opencv2/core/types.hpp>

namespace wmj
{
    AngleSolver::AngleSolver(): m_x_off{0.f}, m_y_off{0.f}, m_z_off{0.f}
    {
        setOffParam();
    }

    /**
     *@brief: 从相机系切换到枪管系
     */
    cv::Point3f AngleSolver::changeCoor(cv::Point3f point)
    {
        cv::Mat trans_mat = GetTransMat(m_x_off, m_y_off, m_z_off);
        cv::Mat target    = (cv::Mat_<float>(4, 1) << point.x, point.y, point.z, 1);
        cv::Mat_<float> target_in_base = trans_mat * target;
        return cv::Point3f(target_in_base.at<float>(0), target_in_base.at<float>(1), target_in_base.at<float>(2));
    }

    /**
     *@brief: 根据装甲板坐标、当前云台位姿、射速等信息计算目标位姿
     */
    GimbalPose AngleSolver::getAngle(cv::Point3f position, GimbalPose cur_pose, double bullet_speed)
    {
        m_bullet_speed = bullet_speed;
        GimbalPose solve_angle;
        cv::Point3f gun_base   = changeCoor(position);
        double pitch_in_camera = -std::atan(gun_base.z / std::sqrt(gun_base.x * gun_base.x + gun_base.y * gun_base.y));
        double yaw_in_camera   = std::atan(gun_base.y / gun_base.x);

        solve_angle.pitch =  cur_pose.pitch + pitch_in_camera + m_pitch_off;
        solve_angle.yaw   =  cur_pose.yaw   + yaw_in_camera   + m_yaw_off;
        if(m_fix_on)
        {
            double dis = sqrt(pow(gun_base.x, 2) + pow(gun_base.z, 2));
            double angle_fix = 0;
            if (abs(9.8 * dis * pow(cos(solve_angle.pitch), 2)) / pow(m_bullet_speed, 2) - sin(solve_angle.pitch) <= 1)
            {
                angle_fix = 0.5 * (asin((9.8 * dis * pow(cos(solve_angle.pitch), 2)) / pow(m_bullet_speed, 2) - sin(solve_angle.pitch)) + solve_angle.pitch);
            }
            else
            {
                std::cout << _warning("[AngleSolver] Wrong distance in angle_fix") << std::endl;
            }
            solve_angle.pitch -= angle_fix;
        }
        return solve_angle;
    }

    GimbalPose AngleSolver::getAngle(cv::Point3f position, GimbalPose cur_pose)
    {
        GimbalPose solve_angle;
        cv::Point3f gun_base   = changeCoor(position);
        double pitch_in_camera = -std::atan(gun_base.z / std::sqrt(gun_base.x * gun_base.x + gun_base.y * gun_base.y));
        double yaw_in_camera   = std::atan(gun_base.y / gun_base.x);

        solve_angle.pitch =  cur_pose.pitch + pitch_in_camera + m_pitch_off;
        solve_angle.yaw   =  cur_pose.yaw   + yaw_in_camera   + m_yaw_off;
        return solve_angle;
    }

    /**
     *@brief: 将装甲板坐标从相对坐标转化为绝对坐标
     */
    cv::Point3f AngleSolver::cam2abs(cv::Point3f position, GimbalPose cur_pose)
    {
        cv::Mat rot_mat    = GetRotMatXYZ(0.0, cur_pose.pitch, cur_pose.yaw);
        cv::Mat trans_mat  = GetTransMat(m_x_off, m_y_off, m_z_off);
        cv::Mat tf_mat     = rot_mat * trans_mat;
        cv::Mat target     = (cv::Mat_<float>(4, 1) << position.x, position.y, position.z, 1);
        cv::Mat abs_target = tf_mat * target;

        return cv::Point3f(abs_target.at<float>(0), abs_target.at<float>(1), abs_target.at<float>(2)); // X Y Z
    }

    /**
     *@brief: 将装甲板坐标从绝对坐标转化为相对坐标
     */
    cv::Point3f AngleSolver::abs2cam(cv::Point3f position, GimbalPose cur_pose)
    {
        cv::Mat rot_mat    = GetRotMatXYZ(0.0, cur_pose.pitch, cur_pose.yaw);
        cv::Mat trans_mat  = GetTransMat(m_x_off, m_y_off, m_z_off);
        cv::Mat tf_mat     = rot_mat * trans_mat;
        cv::Mat target     = (cv::Mat_<float>(4, 1) << position.x, position.y, position.z, 1);
        cv::Mat gun_target = tf_mat.inv() * target;

        return cv::Point3f(gun_target.at<float>(0), gun_target.at<float>(1), gun_target.at<float>(2)); // X Y Z
    }

    cv::Point3f AngleSolver::cam2gun(cv::Point3f position)
    {
        cv::Mat trans_mat  = GetTransMat(m_x_off, m_y_off, m_z_off);
        cv::Mat target     = (cv::Mat_<float>(4, 1) << position.x, position.y, position.z, 1);
        cv::Mat gun_target = trans_mat * target;

        return cv::Point3f(gun_target.at<float>(0), gun_target.at<float>(1), gun_target.at<float>(2));
    }

    void AngleSolver::setOffParam()
    {
        cv::FileStorage fs(ANGLE_CFG, cv::FileStorage::READ);
        fs["x_off"]         >> m_x_off;
        fs["y_off"]         >> m_y_off;
        fs["z_off"]         >> m_z_off;
        fs["pitch_off"]     >> m_pitch_off;
        fs["yaw_off"]       >> m_yaw_off;
        fs["chx_off"]       >> m_chx_off;
        fs["chy_off"]       >> m_chy_off;
        fs["chz_off"]       >> m_chz_off;
        fs["fix_on"]        >> m_fix_on;
        fs.release();
    }

    cv::Mat GetRotMatX(float x)
    {
        return cv::Mat_<float>(4, 4) <<
               1,  0,      0,          0,
               0,  cos(x), -sin(x),    0,
               0,  sin(x), cos(x),     0,
               0,  0,      0,          1;
    }

    cv::Mat GetRotMatY(float y)
    {
        return cv::Mat_<float>(4, 4) <<
               cos(y),     0,  sin(y), 0,
               0,          1,  0,      0,
               -sin(y),    0,  cos(y), 0,
               0,          0,  0,      1;
    }

    cv::Mat GetRotMatZ(float z)
    {
        return cv::Mat_<float>(4, 4) <<
               cos(z), -sin(z),    0,  0,
               sin(z), cos(z),     0,  0,
               0,      0,          1,  0,
               0,      0,          0,  1;
    }

    cv::Mat GetTransMat(float x, float y, float z)
    {
        return cv::Mat_<float>(4, 4) <<
               1,  0,  0,  x,
               0,  1,  0,  y,
               0,  0,  1,  z,
               0,  0,  0,  1;
    }

    cv::Mat GetRotMatXYZ(float x, float y, float z)
    {
        return GetRotMatZ(z) * GetRotMatY(y) * GetRotMatX(x);
    }
}
