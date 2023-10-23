/**
 * 
 * 反陀螺自瞄测试，单双目通用
 * @Author 王铭远，黄永乐
 * 
 * 注：本项目中该文件由于缺乏相关依赖库，无法完成编译，仅作为代码展示
 * 
 */
#include "../libFSM/include/StateMachine.h"


int main(int argc, char **argv)
{
    cv::FileStorage fs("armor_test.yaml", cv::FileStorage::READ);
    int save_point, control_mode;
    fs["save_point"] >> save_point;
    fs["control_mod"] >> control_mode;
    fs.release();

    bool auto_shoot = false;
    double time_start, time_end;
    char c = 'f';
    std::thread(monitorKeyboard, &c).detach();

    // 初始化相机驱动
    std::shared_ptr<wmj::UsbCaptureSystem> camera = std::make_shared<wmj::UsbCaptureSystem>(USBCAPTURE_CFG);
    std::vector<wmj::MatWithTime> frames;
    camera->cameraMode("armor");
    camera->getCameraInfo();
    bool getImage = true;
    int video_number;

    // 初始化识别和控制类
    std::shared_ptr<wmj::WMJRobotControl> control = std::make_shared<wmj::WMJRobotControl>();
    std::shared_ptr<wmj::AngleSolver> angle = std::make_shared<wmj::AngleSolver>();
    std::shared_ptr<wmj::Aimer> aimer = std::make_shared<wmj::Aimer>();
#ifdef USE_DEEP
    std::shared_ptr<wmj::DeepDetector> deep_detector = std::make_shared<wmj::DeepDetector>();
    aimer->setDeepROISize(deep_detector->getInputSize());
#else
    std::shared_ptr<wmj::DoubleDetector> detector = std::make_shared<wmj::DoubleDetector>();
#endif


#ifdef USE_BINO
    video_number = 2;
    frames.resize(video_number);
    frames[0].m_orientation = "left";
    frames[1].m_orientation = "right";
#else
    video_number = 1;
    frames.resize(video_number);
    frames[0].m_orientation = "single";
#endif
    wmj::Rate rate_detect(100);
    wmj::Armor final_armor;
    wmj::GimbalPose target_pose, cur_pose, gm_speed;
    double bullet_speed;
    std::ofstream ofs("../log/Aimer_Record" + std::to_string((int)wmj::now() % 100) + ".txt");

    while (true)
    {
        // 读图
        getImage = true;
        double time_begin = wmj::now();
#ifdef USE_BINO
        if ((*camera >> frames) != 0)
        {
            std::cout << "get images wrong!!!" << std::endl;
            continue;
        }
        for (auto &frame : frames)
        {
            if (frame.m_img.empty())
            {
                getImage = 0;
                std::cout << "Not load empty !!!" << std::endl;
                break;
            }
        }
        if (getImage == 0)
            continue;
        video_number = camera->activeCameraCount();
#else
        if ((*camera >> frames[0]) != 0)
        {
            std::cout << "get images wrong!!!" << std::endl;
            continue;
        }
        if (frames[0].m_img.empty())
        {
            getImage = 0;
            std::cout << "Not load empty !!!" << std::endl;
            continue;
        }
        video_number = camera->activeCameraCount();
#endif
        double time_stop = wmj::now();
        std::cout << "[Aimer] Time cost getImage: " << (time_stop - time_begin) * 1000 << "ms" << std::endl;


        wmj::Armors detect_armors;
        time_start = wmj::now();
        std::vector<cv::Rect2d> rois;
#ifdef USE_BINO
        rois.resize(2);
        // 前一帧ROI
        rois[0] = aimer->getLeftROI();
        rois[1] = aimer->getRightROI();
        detector->DetectArmor(frames, rois);
        detect_armors = detector->toSelector();
#else
        rois.resize(1);
        // 前一帧ROI
        rois[0] = aimer->getLeftROI();

#ifdef USE_DEEP
        deep_detector->DeepDetectSingle(frames[0], rois[0]);
        detect_armors = deep_detector->m_armors;
#else
        detector->DetectArmorSingle(frames, rois);
        detect_armors = detector->toSelector();
#endif
#endif
        // 将测距单位转为m
        for (auto &armor : detect_armors)
        {
            armor.m_position /= 1000;
        }
        time_end = wmj::now();
        std::cout << "[Aimer] Time cost detect: " << (time_end - time_start) * 1000 << "ms" << std::endl;

        // 切换地面系
        control->SwitchBaseCoor(1);

        bullet_speed = control->getShootSpeedValue();
        std::cout << _yellow("[AimerTest] Bullet speed: ") << bullet_speed << std::endl;

        // 获取当前云台位姿
        cur_pose = control->GetGimbalAngle();
        std::cout << _green("[AimerTest] Current pose: ") << cur_pose << std::endl;

        // 录制反陀螺数据
        if (save_point)
        {
            ofs << cur_pose.pitch << " " << cur_pose.yaw << " " << bullet_speed << std::endl;
            for (int i = 0; i < 5; i++)
            {
                if (i < detect_armors.size())
                {
                    ofs << detect_armors[i].m_position.x << " "
                        << detect_armors[i].m_position.y << " "
                        << detect_armors[i].m_position.z << " "
                        << detect_armors[i].m_yaw_angle << std::endl;
                }
                else
                {
                    ofs << "0 0 0 0" << std::endl;
                }
            }
        }

        // Aimer
        target_pose = aimer->getTargetPose(detect_armors, cur_pose, bullet_speed, control_mode);

        if (aimer->isReady() && control_mode != -1)
        {
            // 控制云台
            if (control_mode == 0)
            {
                control->SetGimbalAngle(target_pose.pitch, target_pose.yaw);
                std::cout << _cyan("[Aimer] Target_pose: ") << target_pose << std::endl;
            }
            else if (control_mode == 1)
            {
                control->SetGimbal_YawSpeed_PitchAngle(target_pose.pitch, target_pose.yaw);
                std::cout << _blue("[Aimer] Target_speed: ") << target_pose << std::endl;
            }

            // 发射
            if (auto_shoot)
            {
                if (aimer->shootable())
                {
                    std::cout << _warning("[AimerTest] AUTO SHOOT ONCE") << std::endl;
                    control->ShootOnce();
                }
            }
            else
            {
                control->ShootNone();
            }

            switch (c)
            {
            case '1':
                control->ShootOnce();
                break;
            case '2':
                control->ShootSome(4);
                break;
            case '3':
                control->KeepShoot();
                break;
            case '4':
                control->StopShoot();
                break;
            case '5':
                control->openBox();
                break;
            case 'a':
                auto_shoot = !auto_shoot;
                break;
            default:
                if (!auto_shoot)
                    control->ShootNone();
                break;
            }
        }
        else
        {
            std::cout << _lightred("[AimerTest] Failed to track target") << std::endl;

            control->SetGimbalAngle(cur_pose.pitch, cur_pose.yaw);
            control->ShootNone();
            switch (c)
            {
            case '4':
                control->StopShoot();
                break;
            case '5':
                control->openBox();
                break;
            default:
                break;
            }
        }

        if (auto_shoot)
        {
            std::cout << _lightred("[AimerTest] <<<<<< AUTO SHOOT MODE >>>>>>") << std::endl;
        }
        c = 'f';

#ifdef USE_DEEP
        deep_detector->DebugOutput();
#endif
        std::cout << _warning("************************************") << std::endl;
        rate_detect.sleep();
    }
    return 0;
}
