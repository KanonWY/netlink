#include "nlinktrack_module.h"
#include "init_serial.h"
#include "protocols.h"

#include <chrono>


#define ARRAY_ASSIGN(DEST, SRC)                                                                                                            \
    for (size_t _CNT = 0; _CNT < sizeof(SRC) / sizeof(SRC[0]); ++_CNT)                                                                     \
    {                                                                                                                                      \
        DEST[_CNT] = SRC[_CNT];                                                                                                            \
    }

namespace linktrack
{
    netlink_msg::msg::LinktrackAnchorframe0 g_msg_anchorframe0;
    netlink_msg::msg::LinktrackTagframe0    g_msg_tagframe0;
    netlink_msg::msg::LinktrackNodeframe0   g_msg_nodeframe0;
    netlink_msg::msg::LinktrackNodeframe1   g_msg_nodeframe1;
    netlink_msg::msg::LinktrackNodeframe2   g_msg_nodeframe2;
    netlink_msg::msg::LinktrackNodeframe3   g_msg_nodeframe3;
    netlink_msg::msg::LinktrackNodeframe4   g_msg_nodeframe4;
    netlink_msg::msg::LinktrackNodeframe5   g_msg_nodeframe5;
    netlink_msg::msg::LinktrackNodeframe6   g_msg_nodeframe6;


    NetlinkNode::NetlinkNode()
        : Node("NetLinkNode")
    {
        // init serial
        serial_ = std::make_unique<serial::Serial>();
        initSerial(serial_.get());

        // init NProtocolExtracter
        protocol_extraction_ = std::make_unique<NProtocolExtracter>();

        // init sub
        subscription_ = this->create_subscription<std_msgs::msg::String>("nlink_linktrack_data_transmission", 10,
            [&](const std_msgs::msg::String& msg)
            {
                if (serial_)
                {
                    serial_->write(msg.data);
                }
            });

        // init pub

        publisher_anchrframe0_ = this->create_publisher<netlink_msg::msg::LinktrackAnchorframe0>("nlink_linktrack_anchorframe0", 10);
        publisher_tag_frame0_  = this->create_publisher<netlink_msg::msg::LinktrackTagframe0>("nlink_linktrack_tagframe0", 10);
        publisher_node_frame0_ = this->create_publisher<netlink_msg::msg::LinktrackNodeframe0>("nlink_linktrack_nodeframe0", 10);
        publisher_node_frame1_ = this->create_publisher<netlink_msg::msg::LinktrackNodeframe1>("nlink_linktrack_nodeframe1", 10);
        publisher_node_frame2_ = this->create_publisher<netlink_msg::msg::LinktrackNodeframe2>("nlink_linktrack_nodeframe2", 10);
        publisher_node_frame3_ = this->create_publisher<netlink_msg::msg::LinktrackNodeframe3>("nlink_linktrack_nodeframe3", 10);
        publisher_node_frame4_ = this->create_publisher<netlink_msg::msg::LinktrackNodeframe4>("nlink_linktrack_nodeframe4", 10);
        publisher_node_frame5_ = this->create_publisher<netlink_msg::msg::LinktrackNodeframe5>("nlink_linktrack_nodeframe5", 10);
        publisher_node_frame6_ = this->create_publisher<netlink_msg::msg::LinktrackNodeframe6>("nlink_linktrack_nodeframe6", 10);
    }

#define FREE_NODE(num)                                                                                                                     \
    if (ntl_node_frame_##num##_ptr_)                                                                                                       \
    {                                                                                                                                      \
        delete ntl_node_frame_##num##_ptr_;                                                                                                \
        ntl_node_frame_##num##_ptr_ = nullptr;                                                                                             \
    }

    NetlinkNode::~NetlinkNode()
    {
        if (ntl_anchor_frame0_ptr_)
        {
            delete ntl_anchor_frame0_ptr_;
            ntl_anchor_frame0_ptr_ = nullptr;
        }
        if (ntl_tag_fram0_ptr_)
        {
            delete ntl_tag_fram0_ptr_;
            ntl_tag_fram0_ptr_ = nullptr;
        }
        FREE_NODE(0)
        FREE_NODE(1)
        FREE_NODE(2)
        FREE_NODE(3)
        FREE_NODE(4)
        FREE_NODE(5)
        FREE_NODE(6)
    }

    void NetlinkNode::init()
    {
        initAnchorFrame0();
        initTagFrame0();

        initNodeFrame0();
        initNodeFrame1();
        initNodeFrame2();
        initNodeFrame3();
        initNodeFrame4();
        initNodeFrame5();
        initNodeFrame6();
    }

    void NetlinkNode::initTimer()
    {
        using namespace std::chrono_literals;
        timer_ = this->create_wall_timer(1ms,
            [this]()
            {
                auto        available_bytes = serial_->available();
                std::string str_received;
                if (available_bytes)
                {
                    serial_->read(str_received, available_bytes);
                    // printHexData(str_received);
                    protocol_extraction_->AddNewData(str_received);
                }
            });
    }


    void NetlinkNode::initAnchorFrame0()
    {
        ntl_anchor_frame0_ptr_ = new NLT_ProtocolAnchorFrame0;
        protocol_extraction_->AddProtocol(ntl_anchor_frame0_ptr_);
        ntl_anchor_frame0_ptr_->SetHandleDataCallback(
            [&]
            {
                auto data                      = nlt_anchorframe0_.result;
                g_msg_anchorframe0.role        = data.role;
                g_msg_anchorframe0.id          = data.id;
                g_msg_anchorframe0.voltage     = data.voltage;
                g_msg_anchorframe0.local_time  = data.local_time;
                g_msg_anchorframe0.system_time = data.system_time;
                auto& msg_nodes                = g_msg_anchorframe0.nodes;
                msg_nodes.clear();
                decltype(g_msg_anchorframe0.nodes)::value_type msg_node;
                for (size_t i = 0, icount = data.valid_node_count; i < icount; ++i)
                {
                    auto node     = data.nodes[i];
                    msg_node.role = node->role;
                    msg_node.id   = node->id;
                    ARRAY_ASSIGN(msg_node.pos_3d, node->pos_3d)
                    ARRAY_ASSIGN(msg_node.dis_arr, node->dis_arr)
                    msg_nodes.push_back(msg_node);
                }
                publisher_anchrframe0_->publish(g_msg_anchorframe0);
            });
    }

    void NetlinkNode::initTagFrame0()
    {
        ntl_tag_fram0_ptr_ = new NLT_ProtocolTagFrame0;
        protocol_extraction_->AddProtocol(ntl_tag_fram0_ptr_);
        ntl_tag_fram0_ptr_->SetHandleDataCallback(
            [&]
            {
                const auto& data     = g_nlt_tagframe0.result;
                auto&       msg_data = g_msg_tagframe0;

                msg_data.role        = data.role;
                msg_data.id          = data.id;
                msg_data.local_time  = data.local_time;
                msg_data.system_time = data.system_time;
                msg_data.voltage     = data.voltage;
                ARRAY_ASSIGN(msg_data.pos_3d, data.pos_3d)
                ARRAY_ASSIGN(msg_data.eop_3d, data.eop_3d)
                ARRAY_ASSIGN(msg_data.vel_3d, data.vel_3d)
                ARRAY_ASSIGN(msg_data.dis_arr, data.dis_arr)
                ARRAY_ASSIGN(msg_data.imu_gyro_3d, data.imu_gyro_3d)
                ARRAY_ASSIGN(msg_data.imu_acc_3d, data.imu_acc_3d)
                ARRAY_ASSIGN(msg_data.angle_3d, data.angle_3d)
                ARRAY_ASSIGN(msg_data.quaternion, data.quaternion)

                publisher_tag_frame0_->publish(msg_data);
            });
    }

#define NODEFRAME_INIT(num)                                                                                                                \
    ntl_node_frame_##num##_ptr_ = new NLT_ProtocolNodeFrame##num;                                                                          \
    protocol_extraction_->AddProtocol(ntl_node_frame_##num##_ptr_)

    void NetlinkNode::initNodeFrame0()
    {
        NODEFRAME_INIT(0);
        ntl_node_frame_0_ptr_->SetHandleDataCallback(
            [&]
            {
                const auto& data      = g_nlt_nodeframe0.result;
                auto&       msg_data  = g_msg_nodeframe0;
                auto&       msg_nodes = msg_data.nodes;

                msg_data.role = data.role;
                msg_data.id   = data.id;

                msg_nodes.resize(data.valid_node_count);
                for (size_t i = 0; i < data.valid_node_count; ++i)
                {
                    auto& msg_node = msg_nodes[i];
                    auto  node     = data.nodes[i];
                    msg_node.id    = node->id;
                    msg_node.role  = node->role;
                    msg_node.data.resize(node->data_length);
                    memcpy(msg_node.data.data(), node->data, node->data_length);
                }
                publisher_node_frame0_->publish(msg_data);
            });
    }

    void NetlinkNode::initNodeFrame1()
    {
        NODEFRAME_INIT(1);

        ntl_node_frame_1_ptr_->SetHandleDataCallback(
            [&]
            {
                const auto& data      = g_nlt_nodeframe1.result;
                auto&       msg_data  = g_msg_nodeframe1;
                auto&       msg_nodes = msg_data.nodes;

                msg_data.role        = data.role;
                msg_data.id          = data.id;
                msg_data.local_time  = data.local_time;
                msg_data.system_time = data.system_time;
                msg_data.voltage     = data.voltage;

                msg_nodes.resize(data.valid_node_count);
                for (size_t i = 0; i < data.valid_node_count; ++i)
                {
                    auto& msg_node = msg_nodes[i];
                    auto  node     = data.nodes[i];
                    msg_node.id    = node->id;
                    msg_node.role  = node->role;
                    ARRAY_ASSIGN(msg_node.pos_3d, node->pos_3d)
                }
                publisher_node_frame1_->publish(msg_data);
            });
    }

    void NetlinkNode::initNodeFrame2()
    {
        NODEFRAME_INIT(2);
        ntl_node_frame_2_ptr_->SetHandleDataCallback(
            [&]
            {
                const auto& data      = g_nlt_nodeframe2.result;
                auto&       msg_data  = g_msg_nodeframe2;
                auto&       msg_nodes = msg_data.nodes;

                msg_data.role        = data.role;
                msg_data.id          = data.id;
                msg_data.local_time  = data.local_time;
                msg_data.system_time = data.system_time;
                msg_data.voltage     = data.voltage;
                ARRAY_ASSIGN(msg_data.pos_3d, data.pos_3d)
                ARRAY_ASSIGN(msg_data.eop_3d, data.eop_3d)
                ARRAY_ASSIGN(msg_data.vel_3d, data.vel_3d)
                ARRAY_ASSIGN(msg_data.imu_gyro_3d, data.imu_gyro_3d)
                ARRAY_ASSIGN(msg_data.imu_acc_3d, data.imu_acc_3d)
                ARRAY_ASSIGN(msg_data.angle_3d, data.angle_3d)
                ARRAY_ASSIGN(msg_data.quaternion, data.quaternion)

                msg_nodes.resize(data.valid_node_count);
                for (size_t i = 0; i < data.valid_node_count; ++i)
                {
                    auto& msg_node   = msg_nodes[i];
                    auto  node       = data.nodes[i];
                    msg_node.id      = node->id;
                    msg_node.role    = node->role;
                    msg_node.dis     = node->dis;
                    msg_node.fp_rssi = node->fp_rssi;
                    msg_node.rx_rssi = node->rx_rssi;
                }
                publisher_node_frame2_->publish(msg_data);
            });
    }

    void NetlinkNode::initNodeFrame3()
    {
        NODEFRAME_INIT(3);

        ntl_node_frame_3_ptr_->SetHandleDataCallback(
            [&]
            {
                const auto& data      = g_nlt_nodeframe3.result;
                auto&       msg_data  = g_msg_nodeframe3;
                auto&       msg_nodes = msg_data.nodes;

                msg_data.role        = data.role;
                msg_data.id          = data.id;
                msg_data.local_time  = data.local_time;
                msg_data.system_time = data.system_time;
                msg_data.voltage     = data.voltage;

                msg_nodes.resize(data.valid_node_count);
                for (size_t i = 0; i < data.valid_node_count; ++i)
                {
                    auto& msg_node   = msg_nodes[i];
                    auto  node       = data.nodes[i];
                    msg_node.id      = node->id;
                    msg_node.role    = node->role;
                    msg_node.dis     = node->dis;
                    msg_node.fp_rssi = node->fp_rssi;
                    msg_node.rx_rssi = node->rx_rssi;
                }
                publisher_node_frame3_->publish(msg_data);
            });
    }

    void NetlinkNode::initNodeFrame4()
    {
        NODEFRAME_INIT(4);

        ntl_node_frame_4_ptr_->SetHandleDataCallback(
            [&]
            {
                const auto& data     = g_nlt_nodeframe4.result;
                auto&       msg_data = g_msg_nodeframe4;
                msg_data.role        = data.role;
                msg_data.id          = data.id;
                msg_data.local_time  = data.local_time;
                msg_data.system_time = data.system_time;
                msg_data.voltage     = data.voltage;
                msg_data.tags.resize(data.tag_count);
                for (int i = 0; i < data.tag_count; ++i)
                {
                    auto& msg_tag   = msg_data.tags[i];
                    auto  tag       = data.tags[i];
                    msg_tag.id      = tag->id;
                    msg_tag.voltage = tag->voltage;
                    msg_tag.anchors.resize(tag->anchor_count);
                    for (int j = 0; j < tag->anchor_count; ++j)
                    {
                        auto& msg_anchor = msg_tag.anchors[j];
                        auto  anchor     = tag->anchors[j];
                        msg_anchor.id    = anchor->id;
                        msg_anchor.dis   = anchor->dis;
                    }
                }
                publisher_node_frame4_->publish(msg_data);
            });
    }

    void NetlinkNode::initNodeFrame5()
    {
        NODEFRAME_INIT(5);

        ntl_node_frame_5_ptr_->SetHandleDataCallback(
            [&]
            {
                const auto& data      = g_nlt_nodeframe5.result;
                auto&       msg_data  = g_msg_nodeframe5;
                auto&       msg_nodes = msg_data.nodes;

                msg_data.role        = data.role;
                msg_data.id          = data.id;
                msg_data.local_time  = data.local_time;
                msg_data.system_time = data.system_time;
                msg_data.voltage     = data.voltage;

                msg_nodes.resize(data.valid_node_count);
                for (size_t i = 0; i < data.valid_node_count; ++i)
                {
                    auto& msg_node   = msg_nodes[i];
                    auto  node       = data.nodes[i];
                    msg_node.id      = node->id;
                    msg_node.role    = node->role;
                    msg_node.dis     = node->dis;
                    msg_node.fp_rssi = node->fp_rssi;
                    msg_node.rx_rssi = node->rx_rssi;
                }
                publisher_node_frame5_->publish(msg_data);
            });
    }

    void NetlinkNode::initNodeFrame6()
    {
        NODEFRAME_INIT(6);

        ntl_node_frame_6_ptr_->SetHandleDataCallback(
            [&]
            {
                const auto& data      = g_nlt_nodeframe6.result;
                auto&       msg_data  = g_msg_nodeframe6;
                auto&       msg_nodes = msg_data.nodes;

                msg_data.role = data.role;
                msg_data.id   = data.id;

                msg_nodes.resize(data.valid_node_count);
                for (size_t i = 0; i < data.valid_node_count; ++i)
                {
                    auto& msg_node = msg_nodes[i];
                    auto  node     = data.nodes[i];
                    msg_node.id    = node->id;
                    msg_node.role  = node->role;
                    msg_node.data.resize(node->data_length);
                    memcpy(msg_node.data.data(), node->data, node->data_length);
                }
                publisher_node_frame6_->publish(msg_data);
            });
    }


}  // namespace linktrack
