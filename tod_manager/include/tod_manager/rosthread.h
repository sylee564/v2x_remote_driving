#pragma once
#include <ros/ros.h>
#include <tod_msgs/Status.h>
#include <tod_msgs/ControlCmd.h>
#include <tod_msgs/ProbeVehicleData.h>
#include <atomic>
#include <mutex>
#include <string>
#include <type_traits>

class rosloop_manager {
public:
    rosloop_manager() = default;

    // 프로세스 수명 동안 유지되는 시퀀스(스레드 재시작해도 초기화되지 않음)
    std::atomic<uint32_t> seq_op_{0};
    std::atomic<uint32_t> seq_veh_{0};

    template <class Msg>
    void ros_run_spin_loop(const std::string& node_name,
                           const std::string& topic_name,
                           Msg* msg,
                           bool* ros_terminated,
                           std::mutex* opt_mutex = nullptr,
                           double hz = 20.0,
                           // 외부 공유 카운터(선택). 주면 이것을 사용, 아니면 위 멤버를 사용
                           std::atomic<uint32_t>* seq_op_external  = nullptr,
                           std::atomic<uint32_t>* seq_veh_external = nullptr)
    {
        (void)node_name;

        ros::NodeHandle nh;
        ros::Publisher pub_msg = nh.advertise<std::decay_t<Msg>>(topic_name, 10);
        ros::Rate r(hz);

        using M = std::decay_t<Msg>;
        std::atomic<uint32_t>* sop  = (seq_op_external  ? seq_op_external  : &seq_op_);
        std::atomic<uint32_t>* sveh = (seq_veh_external ? seq_veh_external : &seq_veh_);

        while (ros::ok()) {
            if constexpr (std::is_same<M, tod_msgs::Status>::value) {
                if (opt_mutex) {
                    std::lock_guard<std::mutex> lk(*opt_mutex);
                    const ros::Time now = ros::Time::now();

                    // 퍼블리시 직전에 원자 증가 → 헤더 반영
                    msg->operator_header.seq   = sop->fetch_add(1, std::memory_order_relaxed) + 1;
                    msg->operator_header.stamp = now;

                    msg->vehicle_header.seq    = sveh->fetch_add(1, std::memory_order_relaxed) + 1;
                    msg->vehicle_header.stamp  = now;

                    pub_msg.publish(*msg);
                } else {
                    const ros::Time now = ros::Time::now();

                    msg->operator_header.seq   = sop->fetch_add(1, std::memory_order_relaxed) + 1;
                    msg->operator_header.stamp = now;

                    msg->vehicle_header.seq    = sveh->fetch_add(1, std::memory_order_relaxed) + 1;
                    msg->vehicle_header.stamp  = now;

                    pub_msg.publish(*msg);
                }
            } else {
                if (opt_mutex) {
                    std::lock_guard<std::mutex> lk(*opt_mutex);
                    pub_msg.publish(*msg);
                } else {
                    pub_msg.publish(*msg);
                }
            }

            ros::spinOnce();
            r.sleep();
        }
        if (ros_terminated) *ros_terminated = true;
    }
};
