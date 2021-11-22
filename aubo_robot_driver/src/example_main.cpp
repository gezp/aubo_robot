#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include "AuboRobotMetaType.h"    //机械臂的元数据类型
#include "serviceinterface.h"     //机械臂接口

#define SERVER_HOST "127.0.0.1"
#define SERVER_PORT 8899


class ToolioAndUserIO
{
public:
    ToolioAndUserIO();


public:
    /**
     * @brief run
     *
     * 应用案例：通过工具端IO接按钮实现拖动示教
     *
     *
     * 原理：
     * 　　1. 获取工具端IO指定DI的状态, 进行逻辑判断然后设置用户IO指定DO的状态
     *       如果工具端DI有效　　设置指定用户DO有效
     * 　　　　如果工具端DI无效　　设置指定用户DO无效
     *
     *    2.  用户IO接继电器　 常开触点接SI06和0V　　控制SI06与0V的通断
     */
    static void run();

};

ToolioAndUserIO::ToolioAndUserIO()
{
}

void ToolioAndUserIO::run()
{
    ServiceInterface *robotServicePtr = new ServiceInterface();

    // 接口调用: 连接机械臂服务器
    if(robotServicePtr->robotServiceLogin(SERVER_HOST, SERVER_PORT, "aubo", "123456") != aubo_robot_namespace::InterfaceCallSuccCode)
    {
        std::cerr<<"登录失败."<<std::endl;
        return ;
    }

    //机械臂初始化
//    aubo_robot_namespace::ROBOT_SERVICE_STATE result;
//    aubo_robot_namespace::ToolDynamicsParam toolDynamicsParam;
//    memset(&toolDynamicsParam, 0, sizeof(toolDynamicsParam));
//    robotServicePtr->rootServiceRobotStartup(toolDynamicsParam, 6, true, true, 1000, result, true);

    //获取机械臂诊断信息　　　等待机械臂上电
    aubo_robot_namespace::RobotDiagnosis robotDiagnosisInfo;
    while(true)
    {
        memset(&robotDiagnosisInfo, 0 ,sizeof(robotDiagnosisInfo));

        //接口调用: 获取机械臂诊断信息
        if(robotServicePtr->robotServiceGetRobotDiagnosisInfo(robotDiagnosisInfo)== aubo_robot_namespace::InterfaceCallSuccCode)
        {
            if(robotDiagnosisInfo.armPowerStatus == true)
            {
                break;
            }
        }

        sleep(1);
    }


    bool priToolDIStatus = false;
    bool nowToolDIStatus = false;
    std::vector<aubo_robot_namespace::RobotIoDesc> statusVector;

    while(true)
    {
        statusVector.clear();

        //接口调用: 获取工具端IO的状态
        if(robotServicePtr->robotServiceGetAllToolDigitalIOStatus(statusVector) == aubo_robot_namespace::InterfaceCallSuccCode)
        {
            if(statusVector.size()>1)
            {
                nowToolDIStatus = (statusVector[1].ioValue > 0)? true:false;
            }

            if(nowToolDIStatus != priToolDIStatus)
            {
                //接口调用: 设置用户IO的状态
                robotServicePtr->robotServiceSetBoardIOStatus(aubo_robot_namespace::RobotBoardUserDO, "U_DO_02", (nowToolDIStatus)? 1:0 );
            }

            priToolDIStatus = nowToolDIStatus;
        }
        else
        {
            std::cerr<<"获取工具端IO状态失败."<<std::endl;
        }

        usleep(100*1000);
    }
}

int main(){
    ToolioAndUserIO io;
    io.run();
    return 0;
}