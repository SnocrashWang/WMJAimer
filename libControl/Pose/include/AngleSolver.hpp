#pragma once
#include "../../../libBase/include/common.h"
// #include "../../../libVision/Armor/include/ArmorDetector.hpp"
#include <opencv2/core/types.hpp>

namespace wmj
{
    cv::Mat GetRotMatX(float x);
    cv::Mat GetRotMatY(float y);
    cv::Mat GetRotMatZ(float z);
    cv::Mat GetRotMatXYZ(float x, float y, float z);
    cv::Mat GetTransMat(float x, float y, float z);

    class AngleSolver
    {
    public:
        float m_bullet_speed;

        AngleSolver();
        ~AngleSolver() {}
        void setOffParam();
        cv::Point3f cam2abs(cv::Point3f, GimbalPose);
        cv::Point3f abs2cam(cv::Point3f, GimbalPose);
        cv::Point3f cam2gun(cv::Point3f);
        GimbalPose getAngle(cv::Point3f, GimbalPose, double);
        GimbalPose getAngle(cv::Point3f, GimbalPose);
    private:
        cv::Point3f changeCoor(cv::Point3f point);

        float m_x_off;
        float m_y_off;
        float m_z_off;
        float m_yaw_off;
        float m_pitch_off;
        float m_chx_off;
        float m_chy_off;
        float m_chz_off;
        int   m_fix_on;
    };
}
