//Include Emlid libraries

#include <unistd.h>
#include "Navio2/PWM.h"
#include "Navio+/RCOutput_Navio.h"
#include "Navio2/RCOutput_Navio2.h"
#include "Common/Util.h"
#include <unistd.h>
#include <memory>

//include ROS headers
#include "ros/ros.h"
#include "std_msgs/String.h"

using namespace Navio;

std::unique_ptr<RCOutput> get_rcout()
{
    if (get_navio_version() == NAVIO2)
    {
        auto ptr = std::unique_ptr<RCOutput>{new RCOutput_Navio2()};
        return ptr;
    }
    else
    {
        auto ptr = std::unique_ptr<RCOutput>{new RCOutput_Navio()};
        return ptr;
    }
}

int main(int argc, char **argv)
{
    //initialise based on navio code
    auto pwm = get_rcout();
    if (check_apm())
    {
        return 1;
    }
    if (getuid())
    {
        fprintf(stderr, "Not root. Please launch like this: sudo %s\n", argv[0]);
    }
    if (!(pwm->initialize(PWM_OUTPUT)))
    {
        return 1;
    }

    if (!(pwm->enable(PWM_OUTPUT)))
    {
        return 1;
    }

    pwm->set_frequency(PWM_OUTPUT, 50);

    ros::init(argc, argv, "talker");
    ros::NodeHandle n;
    ros::Publisher chatter_pub = n.advertise<std_msgs::String>("chatter", 1000);

    //Get the parameters
    std::string PWM_OUTPUT;
    nh.get_param("srv_num",PWM_OUTPUT,0);
    std::string PWM_MAX;
    nh.get_param("val_max",PWM_MAX,100);
    std::string PWM_MIN;
    nh.get_param("val_max",PWM_MIN,0);
    std::string TIME_MAX;
    nh.get_param("time_max",TIME_MAX,2.00);
    std::string TIME_MIN;
    nh.get_param("time_min",TIME_MIN,1.00);
    std::string TOPIC;
    nh.get_param("topic",TOPIC,"thrust");


    while (true)
    {
        pwm->set_duty_cycle(PWM_OUTPUT, SERVO_MIN);
        sleep(1);
        pwm->set_duty_cycle(PWM_OUTPUT, SERVO_MAX);
        sleep(1);
    }

    return 0;
}