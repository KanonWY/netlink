#pragma once

#include <nlink_unpack/nlink_utils.h>
#include <protocol_extracter/nprotocol_extracter.h>
#include <rclcpp/rclcpp.hpp>
#include <serial/serial.h>
#include <std_msgs/msg/string.hpp>

#include <netlink_msg/msg/linktrack_anchorframe0.hpp>
#include <netlink_msg/msg/linktrack_nodeframe0.hpp>
#include <netlink_msg/msg/linktrack_nodeframe1.hpp>
#include <netlink_msg/msg/linktrack_nodeframe2.hpp>
#include <netlink_msg/msg/linktrack_nodeframe3.hpp>
#include <netlink_msg/msg/linktrack_nodeframe4.hpp>
#include <netlink_msg/msg/linktrack_nodeframe5.hpp>
#include <netlink_msg/msg/linktrack_nodeframe6.hpp>
#include <netlink_msg/msg/linktrack_tag.hpp>
#include <netlink_msg/msg/linktrack_tagframe0.hpp>

#include <protocols.h>
#include <vector>

namespace linktrack
{
    class NetlinkNode : public rclcpp::Node
    {
    public:
        NetlinkNode();
        virtual ~NetlinkNode() override;
        // virtual ~NetlinkNode();
        void init();

        void initTimer();

        void initAnchorFrame0();
        void initTagFrame0();
        void initNodeFrame0();
        void initNodeFrame1();
        void initNodeFrame2();
        void initNodeFrame3();
        void initNodeFrame4();
        void initNodeFrame5();
        void initNodeFrame6();


    private:
        rclcpp::Publisher<netlink_msg::msg::LinktrackAnchorframe0>::SharedPtr publisher_anchrframe0_;
        rclcpp::Publisher<netlink_msg::msg::LinktrackTagframe0>::SharedPtr    publisher_tag_frame0_;
        rclcpp::Publisher<netlink_msg::msg::LinktrackNodeframe0>::SharedPtr   publisher_node_frame0_;
        rclcpp::Publisher<netlink_msg::msg::LinktrackNodeframe1>::SharedPtr   publisher_node_frame1_;
        rclcpp::Publisher<netlink_msg::msg::LinktrackNodeframe2>::SharedPtr   publisher_node_frame2_;
        rclcpp::Publisher<netlink_msg::msg::LinktrackNodeframe3>::SharedPtr   publisher_node_frame3_;
        rclcpp::Publisher<netlink_msg::msg::LinktrackNodeframe4>::SharedPtr   publisher_node_frame4_;
        rclcpp::Publisher<netlink_msg::msg::LinktrackNodeframe5>::SharedPtr   publisher_node_frame5_;
        rclcpp::Publisher<netlink_msg::msg::LinktrackNodeframe6>::SharedPtr   publisher_node_frame6_;

        rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;

        std::unique_ptr<serial::Serial>     serial_;
        std::unique_ptr<NProtocolExtracter> protocol_extraction_;

        NLT_ProtocolAnchorFrame0* ntl_anchor_frame0_ptr_ = nullptr;
        NLT_ProtocolTagFrame0*    ntl_tag_fram0_ptr_     = nullptr;
        NLT_ProtocolNodeFrame0*   ntl_node_frame_0_ptr_  = nullptr;
        NLT_ProtocolNodeFrame1*   ntl_node_frame_1_ptr_  = nullptr;
        NLT_ProtocolNodeFrame2*   ntl_node_frame_2_ptr_  = nullptr;
        NLT_ProtocolNodeFrame3*   ntl_node_frame_3_ptr_  = nullptr;
        NLT_ProtocolNodeFrame4*   ntl_node_frame_4_ptr_  = nullptr;
        NLT_ProtocolNodeFrame5*   ntl_node_frame_5_ptr_  = nullptr;
        NLT_ProtocolNodeFrame6*   ntl_node_frame_6_ptr_  = nullptr;

    private:
        rclcpp::TimerBase::SharedPtr timer_;
    };
}  // namespace linktrack
