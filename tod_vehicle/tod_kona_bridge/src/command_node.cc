#include "ros/ros.h"

#include "can_msgs/Frame.h"

#include "kona_driver_msgs/PrimaryCommand.h"
#include "kona_driver_msgs/SecondCommand.h"

#include "primary_command_157.hpp"
#include "second_command_156.hpp"

int time_diff = 200000;

static ros::Publisher pub_can;

static can_msgs::Frame can_second;
static can_msgs::Frame can_primary;

static kona_driver_msgs::PrimaryCommand primary_command_msg;
static kona_driver_msgs::SecondCommand second_command_msg;


static Primarycommand157 primary_command;
static Secondcommand156 second_command;

static int primary_prev_t=0, second_prev_t=0;

static void primary_callback(const kona_driver_msgs::PrimaryCommand &msg)
{
    primary_command.Reset();
    primary_command_msg = msg;
    can_primary.header.stamp = ros::Time::now();
    can_primary.dlc = 8;
    primary_command.UpdateData(
        primary_command_msg.steer_angle_target,
        primary_command_msg.accel_dec_cmd,
        primary_command_msg.checksum_157
    );
    can_primary.id = primary_command.ID;
    can_primary.is_extended = false;
    uint8_t *A;
    A = primary_command.get_data();
    
    for(uint i=0;i<8;i++)
    {
        can_primary.data[i] = *A;
        A += 1;
    }
    int t_nsec = 0;
    t_nsec = can_primary.header.stamp.toNSec();
    primary_prev_t = t_nsec;
    // pub_can.publish(can_primary);
}

static void second_callback(const kona_driver_msgs::SecondCommand &msg)
{
    second_command.Reset();
    second_command_msg = msg;
    can_second.header.stamp = ros::Time::now();
    can_second.dlc = 8;
    second_command.UpdateData(
        second_command_msg.eps_en,
        second_command_msg.override_ignore,
        second_command_msg.eps_speed,
        second_command_msg.acc_en,
        second_command_msg.aeb_en,
        second_command_msg.indicator,
        second_command_msg.gear_cmd,
        second_command_msg.checksum_156
    );
    can_second.id = second_command.ID;
    can_second.is_extended= false;
    uint8_t *A;
    A = second_command.get_data();
    for(unsigned int i=0;i<8;i++)
    {
        can_second.data[i] = *A;
        A += 1;
    }
    int t_nsec = 0;
    t_nsec = can_second.header.stamp.toNSec();
    second_prev_t = t_nsec;
    // pub_can.publish(can_second);
}

void timer_callback(const ros::TimerEvent &te)
{
    int time_diff;
    int now;
    now = ros::Time::now().toNSec();
    // primary
    if(now-primary_prev_t>100000000)
    {
        for(uint i=0;i<8;i++)
        {   
            can_primary.id = primary_command.ID;
            can_primary.data[i] = 0;
        }
        // pub_can.publish(can_primary);
    }
    else{
        // pub_can.publish(can_primary);
    }
    // secondary
    if(now-second_prev_t>1000000000)
    {
        for(uint i=0;i<8;i++)
        {   
            can_second.id = second_command.ID;
            can_second.data[i] = 0;
        }
        // pub_can.publish(can_second);
    }
    else{
        // pub_can.publish(can_second);
    }
    
}

int main(int argc, char* argv[])
{
    ros::init(argc, argv, "kona_driver_command_node");
    ros::NodeHandle nh;
    ros::Duration(1).sleep();
    ros::Subscriber sub_primary = nh.subscribe("/Vehicle/kona/primary_command", 1, primary_callback);
    ros::Subscriber sub_second = nh.subscribe("/Vehicle/kona/second_command", 1, second_callback);

    pub_can = nh.advertise<can_msgs::Frame>("/Vehicle/kona/interface/sent_messages", 2, true);
    
    ros::Timer set_speed = nh.createTimer(ros::Duration(0.01), timer_callback);
    ros::spin();

    return 0;
}



