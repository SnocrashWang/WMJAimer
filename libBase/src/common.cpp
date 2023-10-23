#include "../include/common.h"

namespace wmj
{

    Rate::Rate() : Rate(30) {}

    /**
     *@brief: 帧率控制
     *
     *@param: unsigned int 帧率
     *
     *@return: null
     */
    Rate::Rate(unsigned int r)
    :   m_LoopRate(r),
        m_start_time_with_usec(0)
    {}

    /**
     *@brief: 设定时间睡眠以达到帧率控制
     *
     *@param: null
     *
     *@return: void
     */
    void Rate::sleep()
    {
        double period = 1. / m_LoopRate;
        double cur_time = now();
        double expect_end_time = m_start_time_with_usec + period;

        if (expect_end_time > cur_time)
        {
            usleep((expect_end_time - cur_time) * 1000000);
        }
        m_start_time_with_usec = now();
    }

    /**
     *@brief: 返回以微妙为单位的时间戳
     *
     *@param: null
     *
     *@return: void
     */
    unsigned int getTimeUSecNow()
    {
        timeval tv;
        gettimeofday(&tv, NULL);
        return tv.tv_sec * 1000000 + tv.tv_usec;
    }

    /**
     *@brief: 获取当前时间
     *
     *@param: null
     *
     *@return: double 时间戳
     */
    double now()
    {
        timeval tv;
        gettimeofday(&tv, NULL);
        return (double)tv.tv_sec + (double)tv.tv_usec / 1000000;
    }

} //namespace wmj

/**
 *@brief: 终端键盘监听，调试时用这个方便一点
 *
 *@param: 要被监听的char *
 *
 *@return: void
 */
#ifdef Linux
void monitorKeyboard(char *key)
{
    while (true)
    {
        termios new_settings;
        termios stored_settings;
        tcgetattr(0, &stored_settings);
        new_settings = stored_settings;
        new_settings.c_lflag &= (~ICANON);
        new_settings.c_cc[VTIME] = 0;
        tcgetattr(0, &stored_settings);
        new_settings.c_cc[VMIN] = 1;
        tcsetattr(0, TCSANOW, &new_settings);

        *key = getchar();
        tcsetattr(0, TCSANOW, &stored_settings);
    }
}
#endif

/**
 *@brief: 以下均为彩色输出
 *
 *@param: 要输出的字符 std::string
 *
 *@return: 彩色字符 std::string
 */
std::string _red(std::string content)
{
    return "\033[31m" + content + "\033[0m";
}

std::string _lightred(std::string content)
{
    return "\033[1;31m" + content + "\033[0m";
}

std::string _green(std::string content)
{
    return "\033[32m" + content + "\033[0m";
}

std::string _lightgreen(std::string content)
{
    return "\033[1;32m" + content + "\033[0m";
}

std::string _yellow(std::string content)
{
    return "\033[33m" + content + "\033[0m";
}

std::string _brown(std::string content)
{
    return "\033[1;33m" + content + "\033[0m";
}

std::string _blue(std::string content)
{
    return "\033[34m" + content + "\033[0m";
}

std::string _lightblue(std::string content)
{
    return "\033[1;34m" + content + "\033[0m";
}

std::string _purple(std::string content)
{
    return "\033[35m" + content + "\033[0m";
}

std::string _lightpurple(std::string content)
{
    return "\033[1;35m" + content + "\033[0m";
}

std::string _cyan(std::string content)
{
    return "\033[36m" + content + "\033[0m";
}

std::string _lightcyan(std::string content)
{
    return "\033[1;36m" + content + "\033[0m";
}

std::string _white(std::string content)
{
    return "\033[1;37m" + content + "\033[0m";
}

std::string _warning(std::string content)
{
    return "\033[40;33;1;7m" + content + "\033[0m";
}

std::string _underline(std::string content)
{
    return "\033[4m" + content + "\033[0m";
}

std::string _reverse(std::string content)
{
    return "\033[7m" + content + "\033[0m";
}

void _clear()
{
    std::cout << "\033c" << std::endl;
}
