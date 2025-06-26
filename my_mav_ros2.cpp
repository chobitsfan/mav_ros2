#include <stdio.h>
#include <stdlib.h>
#include <sys/time.h>
#include <sys/types.h>
#include <unistd.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <signal.h>
#include <termios.h> // Contains POSIX terminal control definitions
#include <errno.h> // Error integer and strerror() function
#include <math.h>
#include <time.h>
#include "mavlink/common/mavlink.h"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/float32.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "sensor_msgs/msg/range.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "sensor_msgs/msg/point_cloud.hpp"
#include "nav_msgs/msg/odometry.hpp"

using namespace std::chrono_literals;

// ROS coordinate system, x axis = vehicle front

#define MY_COMP_ID MAV_COMP_ID_VISUAL_INERTIAL_ODOMETRY

class MavRosNode : public rclcpp::Node {
    public:
        MavRosNode(int uart_fd) : Node("mavlink_ros"), uart_fd_(uart_fd) {
            odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
                "odometry",
                rclcpp::QoS(1).best_effort().durability_volatile(),
                std::bind(&MavRosNode::odom_callback, this, std::placeholders::_1));
            uart_timer_ = this->create_wall_timer(10ms, [this](){ timer_callback(); });
        }

    private:
        void odom_callback(const nav_msgs::msg::Odometry::UniquePtr odom_msg) {
            struct timespec ts;
            mavlink_message_t msg;
            float covar[21] = {NAN};
            float q[4];
            unsigned int len;
            geometry_msgs::msg::Pose *pose = &odom_msg->pose.pose;
            geometry_msgs::msg::Vector3 *v = &odom_msg->twist.twist.linear;
            if (mav_sysid != 0) {
                q[0] = pose->orientation.w;
                q[1] = pose->orientation.x;
                q[2] = -pose->orientation.y;
                q[3] = -pose->orientation.z;
                clock_gettime(CLOCK_MONOTONIC, &ts);
                //uint64_t now_ms = ts.tv_sec * 1000 + ts.tv_nsec / 1000000;
                //uint64_t odom_ms = odom_msg->header.stamp.sec * 1000 + odom_msg->header.stamp.nanosec / 1000000;
                //int8_t delay_ms = now_ms - odom_ms;
                //uint64_t now_us = ts.tv_sec * 1000000 + ts.tv_nsec / 1000;
                int64_t odom_us = odom_msg->header.stamp.sec * 1000000 + odom_msg->header.stamp.nanosec / 1000;
                mavlink_msg_odometry_pack(mav_sysid, MY_COMP_ID, &msg, odom_us, MAV_FRAME_LOCAL_NED, MAV_FRAME_LOCAL_NED, pose->position.x, -pose->position.y, -pose->position.z, q,
                    v->x, -v->y, -v->z, INFINITY, INFINITY, INFINITY, covar, covar, 0, MAV_ESTIMATOR_TYPE_NAIVE, 0);
                len = mavlink_msg_to_send_buffer(buf, &msg);
                write(uart_fd_, buf, len);
                /*mavlink_msg_att_pos_mocap_pack(mav_sysid, MY_COMP_ID, &msg, now_us, q, pose->position.x, -pose->position.y, -pose->position.z, covar);
                len = mavlink_msg_to_send_buffer(buf, &msg);
                write(uart_fd_, buf, len);
                mavlink_msg_vision_speed_estimate_pack(mav_sysid, MY_COMP_ID, &msg, now_us, v->x, -v->y, -v->z, covar, 0);
                len = mavlink_msg_to_send_buffer(buf, &msg);
                write(uart_fd_, buf, len);*/
            }
        }

        void timer_callback() {
            static int parse_error = 0;
            static int packet_rx_drop_count = 0;
            //struct timeval tv;
            unsigned int len;
            ssize_t avail;
            mavlink_status_t status;
            mavlink_message_t msg;

            memset(&status, 0, sizeof(status));

            avail = read(uart_fd_, buf, 1024);
            for (int i = 0; i < avail; i++) {
                if (mavlink_parse_char(0, buf[i], &msg, &status)) {
                    if (parse_error != status.parse_error) {
                        parse_error = status.parse_error;
                        printf("mavlink parse_error %d\n", parse_error);
                    }
                    if (packet_rx_drop_count != status.packet_rx_drop_count) {
                        packet_rx_drop_count = status.packet_rx_drop_count;
                        printf("mavlink drop %d\n", packet_rx_drop_count);
                    }
                    if (msg.sysid == 255) continue;
                    //printf("recv msg ID %d, seq %d\n", msg.msgid, msg.seq);
                    if (msg.msgid == MAVLINK_MSG_ID_HEARTBEAT) {
                        mavlink_heartbeat_t hb;
                        mavlink_msg_heartbeat_decode(&msg, &hb);
                        if (msg.sysid != mav_sysid) {
                            mav_sysid = msg.sysid;
                            printf("found MAV %d\n", msg.sysid);

                            mavlink_msg_heartbeat_pack(mav_sysid, MY_COMP_ID, &msg, MAV_TYPE_ONBOARD_CONTROLLER, MAV_AUTOPILOT_INVALID, 0, 0, 0);
                            len = mavlink_msg_to_send_buffer(buf, &msg);
                            write(uart_fd_, buf, len);

                            /*gettimeofday(&tv, NULL);
                            mavlink_msg_set_gps_global_origin_pack(mav_sysid, MY_COMP_ID, &msg, mav_sysid, 247749434, 1210443077, 100000, tv.tv_sec*1000000+tv.tv_usec);
                            len = mavlink_msg_to_send_buffer(buf, &msg);
                            write(uart_fd_, buf, len);*/
                        }
                    } else if (msg.msgid == MAVLINK_MSG_ID_STATUSTEXT) {
                        mavlink_statustext_t txt;
                        char txt_buf[64] = {0};
                        mavlink_msg_statustext_decode(&msg, &txt);
                        memcpy(txt_buf, txt.text, 50);
                        printf("fc: %s\n", txt_buf);
                    } else if (msg.msgid == MAVLINK_MSG_ID_TIMESYNC) {
                        struct timespec ts;
                        mavlink_timesync_t sync;
                        mavlink_msg_timesync_decode(&msg, &sync);
                        if (sync.tc1 == 0) {
                            clock_gettime(CLOCK_MONOTONIC, &ts);
                            int64_t ns = ts.tv_sec * 1000000000 + ts.tv_nsec;
                            mavlink_msg_timesync_pack(mav_sysid, MY_COMP_ID, &msg, ns, sync.tc1, mav_sysid, 1);
                        }
                        /*gettimeofday(&tv, NULL);
                        mavlink_msg_system_time_pack(mav_sysid, MY_COMP_ID, &msg, tv.tv_sec*1000000+tv.tv_usec, 0);
                        len = mavlink_msg_to_send_buffer(buf, &msg);
                        write(uart_fd_, buf, len);*/
                    }
                }
            }
        }

        int uart_fd_;
        unsigned char buf[1024];
        uint8_t mav_sysid = 0;
        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
        rclcpp::TimerBase::SharedPtr uart_timer_;
};

int main(int argc, char *argv[]) {
    // Create new termios struc, we call it 'tty' for convention
    struct termios tty;

    rclcpp::init(argc, argv);

    int uart_fd;
    if (argc > 1)
        uart_fd = open(argv[1], O_RDWR| O_NOCTTY | O_NONBLOCK);
    else
        uart_fd = open("/dev/ttyAMA0", O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (uart_fd < 0) {
        printf("can not open serial port\n");
        return 1;
    }

    if(tcgetattr(uart_fd, &tty) != 0) {
        printf("Error %i from tcgetattr: %s\n", errno, strerror(errno));
        return 1;
    }
    tty.c_cflag &= ~PARENB; // Clear parity bit, disabling parity (most common)
    tty.c_cflag &= ~CSTOPB; // Clear stop field, only one stop bit used in communication (most common)
    tty.c_cflag &= ~CSIZE; // Clear all bits that set the data size
    tty.c_cflag |= CS8; // 8 bits per byte (most common)
    tty.c_cflag &= ~CRTSCTS; // Disable RTS/CTS hardware flow control (most common)
    tty.c_cflag |= CREAD | CLOCAL; // Turn on READ & ignore ctrl lines (CLOCAL = 1)
    tty.c_lflag &= ~ICANON;
    tty.c_lflag &= ~ECHO; // Disable echo
    tty.c_lflag &= ~ECHOE; // Disable erasure
    tty.c_lflag &= ~ECHONL; // Disable new-line echo
    tty.c_lflag &= ~ISIG; // Disable interpretation of INTR, QUIT and SUSP
    tty.c_iflag &= ~(IXON | IXOFF | IXANY); // Turn off s/w flow ctrl
    tty.c_iflag &= ~(IGNBRK|BRKINT|PARMRK|ISTRIP|INLCR|IGNCR|ICRNL); // Disable any special handling of received bytes
    tty.c_oflag &= ~OPOST; // Prevent special interpretation of output bytes (e.g. newline chars)
    tty.c_oflag &= ~ONLCR; // Prevent conversion of newline to carriage return/line feed
    tty.c_cc[VTIME] = 10;    // Wait for up to 1s (10 deciseconds), returning as soon as any data is received.
    tty.c_cc[VMIN] = 0;
    cfsetispeed(&tty, B1500000);
    cfsetospeed(&tty, B1500000);
    // Save tty settings, also checking for error
    if (tcsetattr(uart_fd, TCSANOW, &tty) != 0) {
        printf("Error %i from tcsetattr: %s\n", errno, strerror(errno));
        return 1;
    }

    printf("uart ok\n");

    auto node = std::make_shared<MavRosNode>(uart_fd);
    rclcpp::spin(node);
    rclcpp::shutdown();

    close(uart_fd);

    printf("bye\n");

    return 0;
}
