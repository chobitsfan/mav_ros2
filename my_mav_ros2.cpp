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
#include "std_msgs/msg/int64.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "sensor_msgs/msg/range.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "sensor_msgs/msg/point_cloud.hpp"
#include "nav_msgs/msg/odometry.hpp"

using namespace std::chrono_literals;

// ROS coordinate system, x axis = vehicle front

struct MyPoint {
    float x;
    float y;
    float z;
};

float angle_between(float v1_x, float v1_y, float v2_x, float v2_y) {
    float angle1 = atan2f(v1_y, v1_x);
    float angle2 = atan2f(v2_y, v2_x);
    float diff = angle2 - angle1;
    if (diff > M_PI) diff -= 2*M_PI;
    if (diff < -M_PI) diff += 2*M_PI;
    return diff;
}

class MavRosNode : public rclcpp::Node {
    public:
        MavRosNode(int uart_fd) : Node("mavlink_ros"), uart_fd_(uart_fd) {
            odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
                "odometry",
                rclcpp::QoS(1).best_effort().durability_volatile(),
                std::bind(&MavRosNode::odom_callback, this, std::placeholders::_1));
            t_off_sub_ = this->create_subscription<std_msgs::msg::Int64>(
                "pico_pi_t_offset",
                rclcpp::QoS(1).best_effort().durability_volatile(),
                std::bind(&MavRosNode::t_offset_callback, this, std::placeholders::_1));
            uart_timer_ = this->create_wall_timer(10ms, [this](){ timer_callback(); });
        }

    private:
        void t_offset_callback(const std_msgs::msg::Int64::UniquePtr t_offset_msg) {
            pico_pi_t_offset = t_offset_msg->data;
        }
        void odom_callback(const nav_msgs::msg::Odometry::UniquePtr odom_msg) {
            mavlink_message_t msg;
            float nan_cov[21] = {NAN};
            float q[4];
            unsigned int len;
            auto& att = odom_msg->pose.pose.orientation;
            auto& pos = odom_msg->pose.pose.position;
            auto& v = odom_msg->twist.twist.linear;
            auto& cov = odom_msg->pose.covariance;
            if (mav_sysid != 0) {
                q[0] = att.w;
                q[1] = att.x;
                q[2] = -att.y;
                q[3] = -att.z;
                if (is_apm && time_offset_ns != 0) {
                    if (cov[0]+cov[7]+cov[14] > 4 && mode_need_vio) {
                        mavlink_msg_command_long_pack(mav_sysid, MAV_COMP_ID_VISUAL_INERTIAL_ODOMETRY, &msg, 0, 0, MAV_CMD_DO_SET_MODE, 0, 1, 9, 0, 0, 0, 0, 0);
                        len = mavlink_msg_to_send_buffer(buf, &msg);
                        write(uart_fd_, buf, len);
                        RCLCPP_WARN(this->get_logger(), "VIO pose unreliable, land");
                    } else {
                        int64_t odom_fc_us = (odom_msg->header.stamp.sec * 1000000000LL + odom_msg->header.stamp.nanosec - pico_pi_t_offset - time_offset_ns) / 1000;
                        mavlink_msg_att_pos_mocap_pack(mav_sysid, MAV_COMP_ID_VISUAL_INERTIAL_ODOMETRY, &msg, odom_fc_us, q, pos.x, -pos.y, -pos.z, nan_cov);
                        len = mavlink_msg_to_send_buffer(buf, &msg);
                        write(uart_fd_, buf, len);
                        mavlink_msg_vision_speed_estimate_pack(mav_sysid, MAV_COMP_ID_VISUAL_INERTIAL_ODOMETRY, &msg, odom_fc_us, v.x, -v.y, -v.z, nan_cov, 0);
                        len = mavlink_msg_to_send_buffer(buf, &msg);
                        write(uart_fd_, buf, len);
                    }
                } else {
                    int64_t odom_us = odom_msg->header.stamp.sec * 1000000 + odom_msg->header.stamp.nanosec / 1000;
                    mavlink_msg_odometry_pack(mav_sysid, MAV_COMP_ID_VISUAL_INERTIAL_ODOMETRY, &msg, odom_us, MAV_FRAME_LOCAL_NED, MAV_FRAME_LOCAL_NED, pos.x, -pos.y, -pos.z, q,
                        v.x, -v.y, -v.z, INFINITY, INFINITY, INFINITY, nan_cov, nan_cov, 0, MAV_ESTIMATOR_TYPE_NAIVE, 0);
                    len = mavlink_msg_to_send_buffer(buf, &msg);
                    write(uart_fd_, buf, len);
                }
            }
        }

        void timer_callback() {
            static int parse_error = 0;
            static int packet_rx_drop_count = 0;
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

                            mavlink_msg_heartbeat_pack(mav_sysid, MAV_COMP_ID_VISUAL_INERTIAL_ODOMETRY, &msg, MAV_TYPE_ONBOARD_CONTROLLER, MAV_AUTOPILOT_INVALID, 0, 0, 0);
                            len = mavlink_msg_to_send_buffer(buf, &msg);
                            write(uart_fd_, buf, len);

                            struct timespec tp;
                            clock_gettime(CLOCK_MONOTONIC, &tp);
                            mavlink_msg_set_gps_global_origin_pack(mav_sysid, MAV_COMP_ID_VISUAL_INERTIAL_ODOMETRY, &msg, mav_sysid, 247749434, 1210443077, 100000, (uint64_t)tp.tv_sec*1000000+tp.tv_nsec/1000);
                            len = mavlink_msg_to_send_buffer(buf, &msg);
                            write(uart_fd_, buf, len);
                        }
                        if (hb.autopilot == MAV_AUTOPILOT_ARDUPILOTMEGA) {
                            is_apm = true;
                            ts_cnt++;
                            if (ts_cnt > 6) {
                                ts_cnt = 0;
                                struct timespec tp;
                                clock_gettime(CLOCK_MONOTONIC, &tp);
                                mavlink_msg_timesync_pack(mav_sysid, MAV_COMP_ID_VISUAL_INERTIAL_ODOMETRY, &msg, 0, (int64_t)tp.tv_sec * 1000000000 + tp.tv_nsec, mav_sysid, 1);
                                len = mavlink_msg_to_send_buffer(buf, &msg);
                                write(uart_fd_, buf, len);
                            }
                            if (hb.custom_mode == 3 || hb.custom_mode == 4 || hb.custom_mode == 5) mode_need_vio = true; else mode_need_vio = false;
                            if (hb.custom_mode == 4) {
                                if (cur_wp < 0) {
                                    cur_wp = 0;
                                    printf("mission start\n");
                                } else if (cur_wp >= (int)waypoints.size()) {
                                    mavlink_msg_command_long_pack(mav_sysid, MAV_COMP_ID_VISUAL_INERTIAL_ODOMETRY, &msg, 0, 0, MAV_CMD_DO_SET_MODE, 0, 1, 9, 0, 0, 0, 0, 0);
                                    len = mavlink_msg_to_send_buffer(buf, &msg);
                                    write(uart_fd_, buf, len);
                                    printf("mission complete, land\n");
                                }
                            } else cur_wp = -1;
                        }
                        if (!local_pos_rcved) {
                            mavlink_msg_command_long_pack(mav_sysid, MAV_COMP_ID_VISUAL_INERTIAL_ODOMETRY, &msg, mav_sysid, 1, MAV_CMD_SET_MESSAGE_INTERVAL, 0, MAVLINK_MSG_ID_LOCAL_POSITION_NED, 100'000, 0, 0, 0, 0, 0);
                            len = mavlink_msg_to_send_buffer(buf, &msg);
                            write(uart_fd_, buf, len);
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
                        if (sync.tc1 > 0) {
                            time_offset_ns = sync.ts1 - sync.tc1;
                            printf("time offset: %ld ns\n", time_offset_ns);
                        } else if (sync.tc1 == 0) {
                            clock_gettime(CLOCK_MONOTONIC, &ts);
                            int64_t ns = ts.tv_sec * 1000000000 + ts.tv_nsec;
                            mavlink_msg_timesync_pack(mav_sysid, MAV_COMP_ID_VISUAL_INERTIAL_ODOMETRY, &msg, ns, sync.tc1, mav_sysid, 1);
                        }
                        /*gettimeofday(&tv, NULL);
                        mavlink_msg_system_time_pack(mav_sysid, MAV_COMP_ID_VISUAL_INERTIAL_ODOMETRY, &msg, tv.tv_sec*1000000+tv.tv_usec, 0);
                        len = mavlink_msg_to_send_buffer(buf, &msg);
                        write(uart_fd_, buf, len);*/
                    } else if (msg.msgid == MAVLINK_MSG_ID_LOCAL_POSITION_NED) {
                        local_pos_rcved = true;
                        mavlink_local_position_ned_t local_pos;
                        mavlink_msg_local_position_ned_decode(&msg, &local_pos);
                        if (cur_wp >= 0 && cur_wp < (int)waypoints.size()) {
                            float dx = waypoints[cur_wp].x - local_pos.x;
                            float dy = waypoints[cur_wp].y - local_pos.y;
                            if (dx * dx + dy * dy < 1) {
                                printf("waypoint %d arrived\n", cur_wp);
                                ++cur_wp;
                            } else {
                                float tgt_vel_n = 0;
                                float tgt_vel_e = 0;
                                if (local_pos.vx * local_pos.vx + local_pos.vy * local_pos.vy < 0.04f) {
                                    float inv = 1.0f / sqrtf(dx * dx + dy * dy);
                                    tgt_vel_n = dx * inv;
                                    tgt_vel_e = dy * inv;
                                    printf("drone hover: %.2f, %.2f\n", tgt_vel_n, tgt_vel_e);
                                } else {
                                    float tgt_angle = angle_between(local_pos.vx, local_pos.vy, dx, dy);
                                    if (fabsf(tgt_angle) <= (10.0 * M_PI / 180.0)) {
                                        float inv = 1.0f / sqrtf(dx * dx + dy * dy);
                                        tgt_vel_n = dx * inv;
                                        tgt_vel_e = dy * inv;
                                        printf("small angle: %.1f deg, %.2f, %.2f\n", tgt_angle * 180 / M_PI, tgt_vel_n, tgt_vel_e);
                                    } else {
                                        float step;
                                        if (tgt_angle > 0) step = 10.0 * M_PI / 180.0; else step = -10.0 * M_PI / 180.0;
                                        float cos_angle = cosf(step);
                                        float sin_angle = sinf(step);
                                        tgt_vel_n = local_pos.vx * cos_angle - local_pos.vy * sin_angle;
                                        tgt_vel_e = local_pos.vx * sin_angle + local_pos.vy * cos_angle;
                                        float inv = 1.0f / sqrtf(tgt_vel_n * tgt_vel_n + tgt_vel_e * tgt_vel_e);
                                        tgt_vel_n *= inv;
                                        tgt_vel_e *= inv;
                                        printf("angle: %.1f deg, %.2f, %.2f\n", tgt_angle * 180 / M_PI, tgt_vel_n, tgt_vel_e);
                                    }
                                }
                                struct timespec tp;
                                clock_gettime(CLOCK_MONOTONIC, &tp);
                                mavlink_msg_set_position_target_local_ned_pack(mav_sysid, MAV_COMP_ID_VISUAL_INERTIAL_ODOMETRY, &msg, tp.tv_sec*1000+tp.tv_nsec/1000000, mav_sysid, 1, MAV_FRAME_LOCAL_NED, 0xDC7, 0, 0, 0, tgt_vel_n, tgt_vel_e, 0, 0, 0, 0, 0, 0);
                                len = mavlink_msg_to_send_buffer(buf, &msg);
                                write(uart_fd_, buf, len);
                            }
                        }
                    }
                }
            }
        }

        int uart_fd_;
        unsigned char buf[1024];
        uint8_t mav_sysid = 0;
        bool is_apm = false;
        int64_t time_offset_ns = 0;
        int64_t pico_pi_t_offset = 0;
        int ts_cnt = 6;
        bool mode_need_vio = false;
        std::array<MyPoint, 2> waypoints{{{5, 0, -2}, {0, 2, -2}}};
        int cur_wp = -1;
        bool local_pos_rcved = false;
        MyPoint cur_pos;
        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
        rclcpp::Subscription<std_msgs::msg::Int64>::SharedPtr t_off_sub_;
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
