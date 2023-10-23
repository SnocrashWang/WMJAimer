#pragma once
#include <iostream>
#include <string>
#include <thread>
#include <mutex>
#include <functional>
#include <fstream>
#include <memory>

#include <unistd.h>
#include <sys/time.h>
#include <sys/statfs.h>
#include <termio.h>

#include <Eigen/Dense>

#include <opencv2/opencv.hpp>
#include <opencv2/dnn.hpp>

#define PI 3.14159265358
#define HINT                                                        \
    std::cout << _warning("<<<<<\t" + std::string(__FILE__) + ":" + \
                          std::to_string(__LINE__) + "\t>>>>>") +   \
                     "\n";
#ifndef CV_Assert
#define CV_Assert(expr)                                                          \
    do                                                                           \
    {                                                                            \
        if (!!(expr))                                                            \
            ;                                                                    \
        else                                                                     \
            cv::error(cv::Error::StsAssert, #expr, CV_Func, __FILE__, __LINE__); \
    } while (0)
#endif
namespace wmj
{

    unsigned int getTimeUSecNow();

    double now();

    //颜色定义： 红色 _RED = 0, 蓝色 _BLUE = 1, 白色 _WHITE = 2
    enum _COLOR
    {
        _RED = 0,
        _BLUE = 1,
        _WHITE = 2
    };

    /*装甲板大小定义： 未识别 ARMOR_NONE = 0, 小装甲板 ARMOR_SMALL = 1, 大装甲板
     * ARMOR_LARGE = 2*/
    enum ARMORTYPE
    {
        ARMOR_NONE = 0,
        ARMOR_SMALL = 1,
        ARMOR_LARGE = 2,
        ARMOR_RUNE  = 3
    };

    // 装甲板
    class Armor
    {
    public:
        Armor() {};

    public:
        // 装甲板参数
        cv::Point3f m_position;                     // 三维坐标信息
        double m_yaw_angle;                         // 按yaw轴旋转的角度
        double m_time_seq;                          // 时间戳
        int m_id = -1;                              // ID
        wmj::_COLOR m_color;                        // 颜色
        wmj::ARMORTYPE m_armor_type = ARMOR_NONE;   // 装甲板类型，默认ARMOR_NONE
    };
    typedef std::vector<Armor> Armors;

    class Rate
    {
    public:
        Rate();
        Rate(unsigned int);
        double m_start_time_with_usec;

        void sleep();

    private:
        int m_LoopRate;
    };

    // 云台位姿
    struct GimbalPose
    {
        float pitch;
        float yaw;
        float roll;
        double timestamp;

        GimbalPose(float pitch = 0.0, float yaw = 0.0, float roll = 0.0,double timestamp = 0.0)
        {
            this->pitch = pitch;
            this->yaw = yaw;
            this->roll = roll;
            this->timestamp = timestamp;
        }

        GimbalPose operator=(const GimbalPose &gm)
        {
            this->pitch = gm.pitch;
            this->yaw = gm.yaw;
            this->roll = gm.roll;
            this->timestamp = gm.timestamp;
            return *this;
        }

        GimbalPose operator=(const float init_value)
        {
            this->pitch = init_value;
            this->yaw = init_value;
            this->roll = init_value;
            this->timestamp = wmj::now();
            return *this;
        }

        friend GimbalPose operator-(const GimbalPose &gm1, const GimbalPose gm2)
        {
            GimbalPose temp{};
            temp.pitch = gm1.pitch - gm2.pitch;
            temp.yaw = gm1.yaw - gm2.yaw;
            temp.roll = gm1.roll - gm2.roll;
            temp.timestamp = wmj::now();
            return temp;
        }

        friend GimbalPose operator+(const GimbalPose &gm1, const GimbalPose gm2)
        {
            GimbalPose temp{};
            temp.pitch = gm1.pitch + gm2.pitch;
            temp.yaw = gm1.yaw + gm2.yaw;
            temp.roll = gm1.roll + gm2.roll;
            temp.timestamp = wmj::now();
            return temp;
        }

        friend GimbalPose operator*(const GimbalPose &gm, const float k)
        {
            GimbalPose temp{};
            temp.pitch = gm.pitch * k;
            temp.yaw = gm.yaw * k;
            temp.roll = gm.roll * k;
            temp.timestamp = wmj::now();
            return temp;
        }

        friend GimbalPose operator*(const float k, const GimbalPose &gm)
        {

            GimbalPose temp{};
            temp.pitch = gm.pitch * k;
            temp.yaw = gm.yaw * k;
            temp.roll = gm.roll * k;
            temp.timestamp = wmj::now();
            return temp;
        }

        friend GimbalPose operator/(const GimbalPose &gm, const float k)
        {
            GimbalPose temp{};
            temp.pitch = gm.pitch / k;
            temp.yaw = gm.yaw / k;
            temp.roll = gm.roll / k;
            temp.timestamp = wmj::now();
            return temp;
        }

        friend std::ostream &operator<<(std::ostream &out, const GimbalPose &gm)
        {
            out << "[pitch : " << gm.pitch << ", yaw : " << gm.yaw << "]";
            return out;
        }

        float norm()
        {
            return sqrt(pow(this->pitch, 2) + pow(this->yaw, 2));
        }
    };

} // namespace wmj

void monitorKeyboard(char *);

std::string _red(std::string content);
std::string _lightred(std::string content);
std::string _green(std::string content);
std::string _lightgreen(std::string content);
std::string _yellow(std::string content);
std::string _brown(std::string content);
std::string _blue(std::string content);
std::string _lightblue(std::string content);
std::string _purple(std::string content);
std::string _lightpurple(std::string content);
std::string _cyan(std::string content);
std::string _lightcyan(std::string content);
std::string _white(std::string content);
std::string _warning(std::string content);
std::string _underline(std::string content);
std::string _reverse(std::string content);

void _clear();

// rad to degree
inline double R2D(double rad)
{
    return rad * 180 / PI;
}

// degree to rad
inline double D2R(double degree)
{
    return degree * PI / 180;
}

inline double getDistance(cv::Point2d p)
{
    return sqrt(pow(p.x, 2) + pow(p.y, 2));
}

inline double getDistance(cv::Point3d p, bool flag = true)
{
    return sqrt(pow(p.x, 2) + pow(p.y, 2) + pow(p.z, 2) * (int)flag);
}
