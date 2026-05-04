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
#include "mavlink/ardupilotmega/mavlink.h"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/int64.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/vector3.hpp"

using namespace std::chrono_literals;

// ROS coordinate system, x axis = vehicle front

struct MyPoint {
    float x;
    float y;
    float z;
};

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
            lv_sub_ = this->create_subscription<std_msgs::msg::Float32>(
                "light_value",
                rclcpp::QoS(1).best_effort().durability_volatile(),
                std::bind(&MavRosNode::lv_callback, this, std::placeholders::_1));
            tag_dir_sub_ = this->create_subscription<geometry_msgs::msg::Vector3>(
                "tag_dir_frd",
                rclcpp::QoS(1).best_effort().durability_volatile(),
                std::bind(&MavRosNode::tag_dir_callback, this, std::placeholders::_1));
            uart_timer_ = this->create_wall_timer(10ms, [this](){ timer_callback(); });
        }

    private:
        void tag_dir_callback(const geometry_msgs::msg::Vector3::UniquePtr tag_dir) {
            unsigned int len;
            mavlink_message_t msg;
            struct timespec tp;
            clock_gettime(CLOCK_MONOTONIC, &tp);
            tag_found_ts = tp.tv_sec;
            mavlink_msg_set_position_target_local_ned_pack(mav_sysid, MAV_COMP_ID_VISUAL_INERTIAL_ODOMETRY, &msg, tp.tv_sec*1000+tp.tv_nsec/1000000, mav_sysid, 1, MAV_FRAME_BODY_OFFSET_NED, 0xDC7, 0, 0, 0, tag_dir->x*tag_spd, tag_dir->y*tag_spd, tag_dir->z*tag_spd, 0, 0, 0, 0, 0);
            len = mavlink_msg_to_send_buffer(buf, &msg);
            write(uart_fd_, buf, len);
        }
        void lv_callback(const std_msgs::msg::Float32::UniquePtr lv_msg) {
            if (lv_msg->data < low_light_thres) {
                low_light = true;
            } else {
                low_light = false;
            }
        }
        void t_offset_callback(const std_msgs::msg::Int64::UniquePtr t_offset_msg) {
            pico_pi_t_offset = t_offset_msg->data;
        }
        void odom_callback(const nav_msgs::msg::Odometry::UniquePtr odom_msg) {
            mavlink_message_t msg;
            float mav_cov[21] = {NAN};
            float q[4];
            unsigned int len;
            auto& att = odom_msg->pose.pose.orientation;
            auto& pos = odom_msg->pose.pose.position;
            auto& v = odom_msg->twist.twist.linear;
            auto& cov = odom_msg->pose.covariance;
            auto& twist_cov = odom_msg->twist.covariance;
            if (mav_sysid != 0) {
                q[0] = att.w;
                q[1] = att.x;
                q[2] = -att.y;
                q[3] = -att.z;
                if (is_apm) {
                    if (time_offset_ns != 0) {
                        mav_cov[0] = cov[0];
                        mav_cov[6] = cov[7];
                        mav_cov[11] = cov[14];
                        mav_cov[15] = cov[21];
                        mav_cov[18] = cov[28];
                        mav_cov[20] = cov[35];
                        int64_t odom_fc_us = (odom_msg->header.stamp.sec * 1000000000LL + odom_msg->header.stamp.nanosec - pico_pi_t_offset - time_offset_ns) / 1000;
                        mavlink_msg_att_pos_mocap_pack(mav_sysid, MAV_COMP_ID_VISUAL_INERTIAL_ODOMETRY, &msg, odom_fc_us, q, pos.x, -pos.y, -pos.z, mav_cov);
                        len = mavlink_msg_to_send_buffer(buf, &msg);
                        write(uart_fd_, buf, len);
                        mav_cov[0] = twist_cov[0];
                        mav_cov[4] = twist_cov[7];
                        mav_cov[8] = twist_cov[14];
                        mavlink_msg_vision_speed_estimate_pack(mav_sysid, MAV_COMP_ID_VISUAL_INERTIAL_ODOMETRY, &msg, odom_fc_us, v.x, -v.y, -v.z, mav_cov, 0);
                        len = mavlink_msg_to_send_buffer(buf, &msg);
                        write(uart_fd_, buf, len);
                    }
                } else {
                    int64_t odom_us = odom_msg->header.stamp.sec * 1000000 + odom_msg->header.stamp.nanosec / 1000;
                    mavlink_msg_odometry_pack(mav_sysid, MAV_COMP_ID_VISUAL_INERTIAL_ODOMETRY, &msg, odom_us, MAV_FRAME_LOCAL_NED, MAV_FRAME_LOCAL_NED, pos.x, -pos.y, -pos.z, q,
                        v.x, -v.y, -v.z, INFINITY, INFINITY, INFINITY, mav_cov, mav_cov, 0, MAV_ESTIMATOR_TYPE_NAIVE, 0);
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
                            /*mavlink_msg_set_gps_global_origin_pack(mav_sysid, MAV_COMP_ID_VISUAL_INERTIAL_ODOMETRY, &msg, mav_sysid, 247749434, 1210443077, 100000, (uint64_t)tp.tv_sec*1000000+tp.tv_nsec/1000);
                            len = mavlink_msg_to_send_buffer(buf, &msg);
                            write(uart_fd_, buf, len);*/
                            struct timeval tv;
                            gettimeofday(&tv, NULL);
                            mavlink_msg_system_time_pack(mav_sysid, MAV_COMP_ID_VISUAL_INERTIAL_ODOMETRY, &msg, (uint64_t)tv.tv_sec*1000000+tv.tv_usec, tp.tv_sec*1000+tp.tv_nsec/1000000);
                            len = mavlink_msg_to_send_buffer(buf, &msg);
                            write(uart_fd_, buf, len);
                        }
                        /*if (sys_time_not_rcved) {
                            mavlink_msg_command_long_pack(mav_sysid, MAV_COMP_ID_VISUAL_INERTIAL_ODOMETRY, &msg, mav_sysid, 1, MAV_CMD_SET_MESSAGE_INTERVAL, 0, MAVLINK_MSG_ID_SYSTEM_TIME, 200'000, 0, 0
, 0, 0, 0);
                            len = mavlink_msg_to_send_buffer(buf, &msg);
                            write(uart_fd_, buf, len);
                        }*/
                        if (gps_raw_not_rcved) {
                            mavlink_msg_command_long_pack(mav_sysid, MAV_COMP_ID_VISUAL_INERTIAL_ODOMETRY, &msg, mav_sysid, 1, MAV_CMD_SET_MESSAGE_INTERVAL, 0, MAVLINK_MSG_ID_GPS_RAW_INT, 1'000'000, 0, 0
, 0, 0, 0);
                            len = mavlink_msg_to_send_buffer(buf, &msg);
                            write(uart_fd_, buf, len);
                        }
                        if (ekf_status_not_rcved) {
                            mavlink_msg_command_long_pack(mav_sysid, MAV_COMP_ID_VISUAL_INERTIAL_ODOMETRY, &msg, mav_sysid, 1, MAV_CMD_SET_MESSAGE_INTERVAL, 0, MAVLINK_MSG_ID_EKF_STATUS_REPORT, 200'000, 0, 0, 0, 0, 0);
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
                            cur_mode = hb.custom_mode;
                            if (hb.custom_mode == 4) { // guided
#if 1
                                if (act_idx == 0) {
                                    act_idx = 1;
                                    struct timespec tp;
                                    clock_gettime(CLOCK_MONOTONIC, &tp);
                                    mavlink_msg_set_position_target_local_ned_pack(mav_sysid, MAV_COMP_ID_VISUAL_INERTIAL_ODOMETRY, &msg, tp.tv_sec*1000+tp.tv_nsec/1000000, mav_sysid, 1, MAV_FRAME_BODY_OFFSET_NED, 0x0DF8, 10, 0, -0.1f, 0, 0, 0, 0, 0, 0, 0, 0);
                                    len = mavlink_msg_to_send_buffer(buf, &msg);
                                    write(uart_fd_, buf, len);
                                    RCLCPP_INFO(this->get_logger(), "mission start");
                                } else if (act_idx == 1) {
                                    struct timespec tp;
                                    clock_gettime(CLOCK_MONOTONIC, &tp);
                                    if (tp.tv_sec - tag_found_ts < 1) {
                                        act_idx = 2;
                                        brake_cnt = 2; // give brake some time to stop vehicle
                                        mavlink_msg_command_long_pack(mav_sysid, MAV_COMP_ID_VISUAL_INERTIAL_ODOMETRY, &msg, 0, 0, MAV_CMD_DO_SET_MODE, 0, 1, 17, 0, 0, 0, 0, 0); // brake
                                        len = mavlink_msg_to_send_buffer(buf, &msg);
                                        write(uart_fd_, buf, len);
                                        RCLCPP_INFO(this->get_logger(), "brake");
                                    }
                                } else if (act_idx == 4) {
                                    act_idx = 5;
                                    struct timespec tp;
                                    clock_gettime(CLOCK_MONOTONIC, &tp);
                                    mavlink_msg_set_position_target_local_ned_pack(mav_sysid, MAV_COMP_ID_VISUAL_INERTIAL_ODOMETRY, &msg, tp.tv_sec*1000+tp.tv_nsec/1000000, mav_sysid, 1, MAV_FRAME_BODY_OFFSET_NED, 0x0DF8, 5, 0, -0.1f, 0, 0, 0, 0, 0, 0, 0, 0);
                                    len = mavlink_msg_to_send_buffer(buf, &msg);
                                    write(uart_fd_, buf, len);
                                    RCLCPP_INFO(this->get_logger(), "forward a little");
                                } else if (act_idx == 5) {
                                    if (gps_not_avail) {
                                        act_idx = 6;
                                        mavlink_msg_command_long_pack(mav_sysid, MAV_COMP_ID_VISUAL_INERTIAL_ODOMETRY, &msg, 0, 0, MAV_CMD_DO_SET_MODE, 0, 1, 9, 0, 0, 0, 0, 0); // land
                                        len = mavlink_msg_to_send_buffer(buf, &msg);
                                        write(uart_fd_, buf, len);
                                        RCLCPP_INFO(this->get_logger(), "mission finished");
                                    }
                                }
#endif
                            } else if (hb.custom_mode == 17) { // brake
                                if (act_idx == 2) {
                                    --brake_cnt;
                                    if (brake_cnt < 0) {
                                        act_idx = 3;
                                        latest_yaw_reset_ms = 0;
                                        mavlink_msg_command_int_pack(mav_sysid, MAV_COMP_ID_VISUAL_INERTIAL_ODOMETRY, &msg, mav_sysid, 1, MAV_FRAME_GLOBAL, 42007, 0, 0, 2, 0, 0, 0, 0, 0, 0);
                                        len = mavlink_msg_to_send_buffer(buf, &msg);
                                        write(uart_fd_, buf, len);
                                        RCLCPP_INFO(this->get_logger(), "ekf switch to src2");
                                    }
                                } else if (act_idx == 3) {
                                    struct timespec tp;
                                    clock_gettime(CLOCK_MONOTONIC, &tp);
                                    int now_ms = tp.tv_sec * 1000 + tp.tv_nsec / 1000000;
                                    if (now_ms - latest_yaw_reset_ms > 200) { // wait pos reset done
                                        act_idx = 4;
                                        mavlink_msg_command_long_pack(mav_sysid, MAV_COMP_ID_VISUAL_INERTIAL_ODOMETRY, &msg, 0, 0, MAV_CMD_DO_SET_MODE, 0, 1, 4, 0, 0, 0, 0, 0); // guided
                                        len = mavlink_msg_to_send_buffer(buf, &msg);
                                        write(uart_fd_, buf, len);
                                        RCLCPP_INFO(this->get_logger(), "ekf yaw reset, carry on");
                                    }
                                }
                            } else {
                                act_idx = 0;
                            }
                        }
                    } else if (msg.msgid == MAVLINK_MSG_ID_STATUSTEXT) {
                        mavlink_statustext_t txt;
                        char txt_buf[64] = {0};
                        mavlink_msg_statustext_decode(&msg, &txt);
                        memcpy(txt_buf, txt.text, 50);
                        printf("FC: %s\n", txt_buf);
                    } else if (msg.msgid == MAVLINK_MSG_ID_EVENT) {
                        mavlink_event_t evt;
                        mavlink_msg_event_decode(&msg, &evt);
                        if (evt.event_time_boot_ms != latest_evt_ms) {
                            latest_evt_ms = evt.event_time_boot_ms;
                            //printf("event: %d %d\n", evt.id, evt.event_time_boot_ms);
                            if (evt.id == 62) { // EKF_YAW_RESET
                                struct timespec tp;
                                clock_gettime(CLOCK_MONOTONIC, &tp);
                                latest_yaw_reset_ms = tp.tv_sec * 1000 + tp.tv_nsec / 1000000;
                                RCLCPP_INFO(this->get_logger(), "ekf yaw reset at %d ms", evt.event_time_boot_ms);
                            } else if (evt.id == 18) { // LAND_COMPLETE
                                not_landed = false;
                            } else if (evt.id == 28) { // NOT_LANDED
                                not_landed = true;
                            }
                        }
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
                    } else if (msg.msgid == MAVLINK_MSG_ID_GPS_RAW_INT) {
                        gps_raw_not_rcved = false;
                        mavlink_gps_raw_int_t gps_raw;
                        mavlink_msg_gps_raw_int_decode(&msg, &gps_raw);
                        //printf("hdop:%d, spd acc:%d\n", gps_raw.eph, gps_raw.vel_acc);
                        if (gps_raw.vel_acc > 2000) gps_not_avail = true; else gps_not_avail = false;
                    } else if (msg.msgid == MAVLINK_MSG_ID_EKF_STATUS_REPORT) {
                        ekf_status_not_rcved = false;
                        mavlink_ekf_status_report_t ekf_status;
                        mavlink_msg_ekf_status_report_decode(&msg, &ekf_status);
                        //if (cur_mode == COPTER_MODE_LOITER && not_landed) printf("mag var:%f\n", ekf_status.compass_variance);
                        if (prv_mag_bad && ekf_status.compass_variance > 0.9) mag_bad = true; else mag_bad = false;
                        if (ekf_status.compass_variance > 0.9) prv_mag_bad = true; else prv_mag_bad = false;
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
        uint32_t latest_evt_ms = 0;
        int latest_yaw_reset_ms = 0;
        //bool sys_time_not_rcved = true;
        bool gps_raw_not_rcved = true;
        bool low_light = false;
        int act_idx = 0;
        bool gps_not_avail = true;
        bool ekf_status_not_rcved = true;
        bool mag_bad = false;
        bool prv_mag_bad = false;
        bool not_landed = false;
        int tag_found_ts = 0;
        uint32_t cur_mode = 0;
        constexpr static float low_light_thres = 11;
        constexpr static float tag_spd = 0.5;
        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
        rclcpp::Subscription<std_msgs::msg::Int64>::SharedPtr t_off_sub_;
        rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr lv_sub_;
        rclcpp::Subscription<geometry_msgs::msg::Vector3>::SharedPtr tag_dir_sub_;
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
