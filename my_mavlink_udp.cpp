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
#include <sys/socket.h>
#include <sys/un.h>
#include <arpa/inet.h>
#include <netinet/in.h>
#include <math.h>
#include <sys/wait.h>
#include <poll.h>
#include <time.h>
#include "mavlink/ardupilotmega/mavlink.h"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/float32.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "geometry_msgs/msg/point.hpp"

// ROS coordinate system, x axis = vehicle front

#define MY_COMP_ID 191
#define MY_NUM_PFDS 3
#define SERVER_PATH "/tmp/chobits_server"
#define SERVER_PATH2 "/tmp/chobits_server2"

#define MAX_WP_DIST_M 8

#define SEARCH_STRUCT_CROSS 1
#define PASS_STRUCT_CROSS 2
#define MOVE_UP 1
#define MOVE_RIGHT 2
#define MOVE_DOWN 3
#define MOVE_LEFT 4
#define LAND 5
#define HOVER 6

#define CLOSE_DIST_M 0.5
#define FAR_DIST_M 0.9

#define TEMP_MATCHING

bool cog_ok = false;
unsigned int no_cog_c = 0;

struct __attribute__((packed)) lines_3d {
//where (vx, vy, vz) is a normalized vector collinear to the line and (x0, y0, z0) is a point on the line.
    float vert_x;
    float vert_y;
    float vert_z;
    float vert_vx;
    float vert_vy;
    float vert_vz;
    float hori_x;
    float hori_y;
    float hori_z;
    float hori_vx;
    float hori_vy;
    float hori_vz;
};

float angle_between_vectors(float v1x, float v1y, float v1z, float v2x, float v2y, float v2z) {
    return acosf((v1x * v2x + v1y * v2y + v1z * v2z) / (sqrtf(v1x * v1x + v1y * v1y + v1z * v1z) * sqrtf(v2x * v2x + v2y * v2y + v2z * v2z)));
}

void cog_callback(const geometry_msgs::msg::Point::SharedPtr msg) {
    //printf("cog %f %f\n", msg->x, msg->y);
    if (msg->x >= 0 && msg->y >= 0) {
        cog_ok = true;
        no_cog_c = 0;
    } else {
        no_cog_c++;
        cog_ok = false;
    }
}

int main(int argc, char *argv[]) {
    struct pollfd pfds[MY_NUM_PFDS];
    struct timeval tv;
    int retval, uart_fd;
    unsigned int len;
    unsigned char buf[1024];
    ssize_t avail;
    mavlink_status_t status;
    mavlink_message_t msg;
    // Create new termios struc, we call it 'tty' for convention
    struct termios tty;
    struct sockaddr_un ipc_addr, ipc_addr2;
    uint8_t mav_sysid = 0;
    int ipc_fd, ipc_fd2;
    int parse_error = 0, packet_rx_drop_count = 0;
    bool att_rcved = false;
    float cur_yaw = 0;
    int missions[] = {
        MOVE_UP,
        MOVE_LEFT,
        MOVE_DOWN,
        MOVE_LEFT,
        MOVE_UP,
        MOVE_RIGHT,
        MOVE_DOWN,
        MOVE_RIGHT,
        MOVE_UP,
        MOVE_LEFT,
        MOVE_DOWN,
        MOVE_LEFT,
        MOVE_UP,
        MOVE_RIGHT,
        MOVE_DOWN,
        MOVE_RIGHT,
        MOVE_UP,
        MOVE_LEFT,
        MOVE_DOWN,
        MOVE_LEFT,
        MOVE_UP,
        MOVE_RIGHT,
        MOVE_DOWN,
        MOVE_RIGHT,
        LAND,
    };
    int mission_idx = -1;
    float cur_vio_x = 0, cur_vio_y = 0, cur_vio_z = 0;
    struct lines_3d detected_structs;
    struct lines_3d last_detected_structs;
    int confirm_cnt = 0;
    int navi_status = SEARCH_STRUCT_CROSS;
    int move_status = HOVER;
    int low_confirm_cnt = 0;
    int high_confirm_cnt = 0;
    int close_confirm_cnt = 0;
    int far_confirm_cnt = 0;
    int align_confirm_cnt = 0;
    int right_confirm_cnt = 0;
    int left_confirm_cnt = 0;
    int yaw_adj_cd = 0;
    float tgt_yaw = 0;
    unsigned int adj_cnt = 0;
    unsigned int cross_id = 0;

    visualization_msgs::msg::Marker line_list;
    line_list.header.frame_id = "body";
    line_list.type = visualization_msgs::msg::Marker::LINE_LIST;
    line_list.action = visualization_msgs::msg::Marker::ADD;
    line_list.pose.orientation.w = 1.0;
    line_list.id = 1;
    line_list.scale.x = 0.02;
    line_list.color.r = 1.0;
    line_list.color.g = 1.0;
    line_list.color.b = 1.0;
    line_list.color.a = 1.0;

    memset(&last_detected_structs, 0, sizeof(last_detected_structs));

    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("mavlink_udp");
    auto navi_pub = node->create_publisher<std_msgs::msg::String>("navi", 1);
    auto vis_pub = node->create_publisher<visualization_msgs::msg::Marker>("struct_lines", 1);
    auto roll_pub = node->create_publisher<std_msgs::msg::Float32>("roll", 1);
    auto cog_sub = node->create_subscription<geometry_msgs::msg::Point>("templateCOG", 1, cog_callback);

    if (argc > 1)
        uart_fd = open(argv[1], O_RDWR| O_NOCTTY);
    else
        uart_fd = open("/dev/ttyAMA0", O_RDWR | O_NOCTTY);
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

    if ((ipc_fd = socket(AF_UNIX, SOCK_DGRAM, 0)) < 0) {
        return 1;
    }
    memset(&ipc_addr, 0, sizeof(ipc_addr));
    ipc_addr.sun_family = AF_UNIX;
    strcpy(ipc_addr.sun_path, SERVER_PATH);
    unlink(SERVER_PATH);
    if (bind(ipc_fd, (const struct sockaddr *)&ipc_addr, sizeof(ipc_addr)) < 0) {
        printf("bind local failed\n");
        return 1;
    }

    if ((ipc_fd2 = socket(AF_UNIX, SOCK_DGRAM, 0)) < 0) {
        return 1;
    }
    memset(&ipc_addr2, 0, sizeof(ipc_addr2));
    ipc_addr2.sun_family = AF_UNIX;
    strcpy(ipc_addr2.sun_path, SERVER_PATH2);
    unlink(SERVER_PATH2);
    if (bind(ipc_fd2, (const struct sockaddr *)&ipc_addr2, sizeof(ipc_addr2)) < 0) {
        printf("bind local failed\n");
        return 1;
    }

    pfds[0].fd= uart_fd;
    pfds[0].events = POLLIN;
    pfds[1].fd= ipc_fd;
    pfds[1].events = POLLIN;
    pfds[2].fd= ipc_fd2;
    pfds[2].events = POLLIN;

    printf("hello\n");

    memset(&status, 0, sizeof(status));

    while (rclcpp::ok()) {
        rclcpp::spin_some(node);
        retval = poll(pfds, MY_NUM_PFDS, 10);
        if (retval > 0) {
            if (pfds[0].revents & POLLIN) {
                avail = read(uart_fd, buf, 1024);
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

                                gettimeofday(&tv, NULL);
                                mavlink_msg_system_time_pack(mav_sysid, MY_COMP_ID, &msg, tv.tv_sec*1000000+tv.tv_usec, 0);
                                len = mavlink_msg_to_send_buffer(buf, &msg);
                                write(uart_fd, buf, len);

                                mavlink_msg_set_gps_global_origin_pack(mav_sysid, MY_COMP_ID, &msg, mav_sysid, 247749434, 1210443077, 100000, tv.tv_sec*1000000+tv.tv_usec);
                                len = mavlink_msg_to_send_buffer(buf, &msg);
                                write(uart_fd, buf, len);
                            }
                            if (hb.custom_mode == COPTER_MODE_GUIDED) {
                                if (mission_idx == -1) {
                                    auto txt = std_msgs::msg::String();
                                    txt.data = "mission start";
                                    navi_pub->publish(txt);
                                    mission_idx = 0;
                                    navi_status = SEARCH_STRUCT_CROSS;
                                    move_status = HOVER;
                                } /*else {
                                    float x_diff = cur_vio_x - wp_vio_x;
                                    float y_diff = cur_vio_y - wp_vio_y;
                                    float z_diff = cur_vio_z - wp_vio_z;
                                    if ((x_diff * x_diff + y_diff * y_diff + z_diff * z_diff) > (MAX_WP_DIST_M * MAX_WP_DIST_M)) {
                                        printf("exceed MAX_WP_DIST_M, abort mission\n");
                                        mavlink_msg_set_mode_pack(mav_sysid, MY_COMP_ID, &msg, mav_sysid, 1, 9); //land
                                        len = mavlink_msg_to_send_buffer(buf, &msg);
                                        write(uart_fd, buf, len);
                                    }
                                }*/
                            } else {
                                mission_idx = -1;
                            }
                            if (!att_rcved) {
                                mavlink_msg_command_long_pack(mav_sysid, MY_COMP_ID, &msg, 0, 0, MAV_CMD_SET_MESSAGE_INTERVAL, 0, MAVLINK_MSG_ID_ATTITUDE, 100000, 0, 0, 0, 0, 0);
                                len = mavlink_msg_to_send_buffer(buf, &msg);
                                write(uart_fd, buf, len);
                            }
                        } else if (msg.msgid == MAVLINK_MSG_ID_STATUSTEXT) {
                            mavlink_statustext_t txt;
                            mavlink_msg_statustext_decode(&msg, &txt);
                            printf("fc: %s\n", txt.text);
                        } else if (msg.msgid == MAVLINK_MSG_ID_ATTITUDE) {
                            att_rcved = true;
                            mavlink_attitude_t att;
                            mavlink_msg_attitude_decode(&msg, &att);
                            auto m = std_msgs::msg::Float32();
                            m.data = att.roll;
                            roll_pub->publish(m);
                        }
                    }
                }
            }
            if (pfds[1].revents & POLLIN) {
                float pose[10];
                if (recv(ipc_fd, pose, sizeof(pose), 0) > 0) {
                    float covar[21] = {0};
                    pose[2]=-pose[2];
                    pose[3]=-pose[3];
                    if (mav_sysid != 0) {
                        gettimeofday(&tv, NULL);
                        mavlink_msg_att_pos_mocap_pack(mav_sysid, MY_COMP_ID, &msg, tv.tv_sec*1000000+tv.tv_usec, pose, pose[4], -pose[5], -pose[6], covar);
                        len = mavlink_msg_to_send_buffer(buf, &msg);
                        write(uart_fd, buf, len);
                        gettimeofday(&tv, NULL);
                        mavlink_msg_vision_speed_estimate_pack(mav_sysid, MY_COMP_ID, &msg, tv.tv_sec*1000000+tv.tv_usec, pose[7], -pose[8], -pose[9], covar, 0);
                        len = mavlink_msg_to_send_buffer(buf, &msg);
                        write(uart_fd, buf, len);
                    }
                    cur_vio_x = pose[4];
                    cur_vio_y = pose[5];
                    cur_vio_z = pose[6];
                }
            }
            if (pfds[2].revents & POLLIN) {
                if (recv(ipc_fd2, &detected_structs, sizeof(detected_structs), 0) > 0) {
                    /*if (detected_structs.hori_x != 0) {
                        float vx, vy;
                        if (detected_structs.hori_vy < 0) {
                            vx = -detected_structs.hori_vx;
                            vy = -detected_structs.hori_vy;
                        } else {
                            vx = detected_structs.hori_vx;
                            vy = detected_structs.hori_vy;
                        }
                        float angle_y_hori = acosf(vy);
                        if (angle_y_hori > 0.15) align_confirm_cnt++; else align_confirm_cnt = 0;
                        if (align_confirm_cnt > 2) {
                            align_confirm_cnt = 0;
                            printf("angle_y_hori %f %f\n", angle_y_hori, vx);
                        }
                    }*/
                    if (yaw_adj_cd > 0) yaw_adj_cd--;
                    if (mission_idx >= 0 && (unsigned int)mission_idx < (sizeof(missions) / sizeof(missions[0]))) {
                        if (navi_status == SEARCH_STRUCT_CROSS) {
#ifdef TEMP_MATCHING
                            if (cog_ok) {
                                auto txt = std_msgs::msg::String();
                                txt.data = "cross detected, mission idx " + std::to_string(mission_idx);
                                navi_pub->publish(txt);
                                navi_status = PASS_STRUCT_CROSS;
                                move_status = missions[mission_idx];
                            }
#else
                            if (detected_structs.vert_x != 0 && detected_structs.hori_x != 0) confirm_cnt++;
                            if (confirm_cnt > 2) {
                                auto txt = std_msgs::msg::String();
                                txt.data = "cross detected, mission idx " + std::to_string(mission_idx);
                                navi_pub->publish(txt);
                                confirm_cnt = 0;
                                navi_status = PASS_STRUCT_CROSS;
                                move_status = missions[mission_idx];

                                line_list.header.stamp = node->get_clock()->now();
                                cross_id++;
                                line_list.id = cross_id;
                                line_list.ns = "cross";
                                line_list.color.r = line_list.color.g = line_list.color.b = 0.5;
                                line_list.points.clear();
                                geometry_msgs::msg::Point p;
                                p.x = detected_structs.hori_x + detected_structs.hori_vx;
                                p.y = detected_structs.hori_y + detected_structs.hori_vy;
                                p.z = detected_structs.hori_z + detected_structs.hori_vz;
                                line_list.points.push_back(p);
                                p.x = detected_structs.hori_x - detected_structs.hori_vx;
                                p.y = detected_structs.hori_y - detected_structs.hori_vy;
                                p.z = detected_structs.hori_z - detected_structs.hori_vz;
                                line_list.points.push_back(p);
                                p.x = detected_structs.vert_x + detected_structs.vert_vx;
                                p.y = detected_structs.vert_y + detected_structs.vert_vy;
                                p.z = detected_structs.vert_z + detected_structs.vert_vz;
                                line_list.points.push_back(p);
                                p.x = detected_structs.vert_x - detected_structs.vert_vx;
                                p.y = detected_structs.vert_y - detected_structs.vert_vy;
                                p.z = detected_structs.vert_z - detected_structs.vert_vz;
                                line_list.points.push_back(p);
                                vis_pub->publish(line_list);
                            }
#endif
                        } else if (navi_status == PASS_STRUCT_CROSS) {
#ifdef TEMP_MATCHING
                            if (no_cog_c > 8) {
                                auto txt = std_msgs::msg::String();
                                txt.data = "cross passed";
                                navi_pub->publish(txt);
                                navi_status = SEARCH_STRUCT_CROSS;
                                if (mission_idx >= 0) mission_idx++;
                            }
#else
                            if (detected_structs.vert_x == 0 || detected_structs.hori_x == 0) confirm_cnt++;
                            if (confirm_cnt > 2) {
                                auto txt = std_msgs::msg::String();
                                txt.data = "cross passed";
                                navi_pub->publish(txt);
                                confirm_cnt = 0;
                                navi_status = SEARCH_STRUCT_CROSS;
                                if (mission_idx >= 0) mission_idx++;
                            }
#endif
                        }
                        if (move_status == MOVE_RIGHT || move_status == MOVE_LEFT) {
                            float vel_r = 0.2;
                            float vel_d = 0;
                            float vel_f = 0;
                            uint16_t type_mask = 0xdc7;
                            if (move_status == MOVE_LEFT) vel_r = -0.2;
                            if (detected_structs.hori_x == 0) {
                            } else {
                                float t = -detected_structs.hori_y / detected_structs.hori_vy;
                                float z = detected_structs.hori_z + detected_structs.hori_vz * t;
                                float x = detected_structs.hori_x + detected_structs.hori_vx * t;
                                if (z > 0.2) low_confirm_cnt++; else low_confirm_cnt = 0;
                                if (z < -0.2) high_confirm_cnt++; else high_confirm_cnt = 0;
                                if (x < CLOSE_DIST_M) close_confirm_cnt++; else close_confirm_cnt = 0;
                                if (x > FAR_DIST_M) far_confirm_cnt++; else far_confirm_cnt = 0;
                                if (low_confirm_cnt > 2) {
                                    vel_d = -0.1;
                                    auto txt = std_msgs::msg::String();
                                    txt.data = "too low, move up";
                                    navi_pub->publish(txt);
                                } else if (high_confirm_cnt > 2) {
                                    vel_d = 0.1;
                                    auto txt = std_msgs::msg::String();
                                    txt.data = "too high, move down";
                                    navi_pub->publish(txt);
                                }
                                if (close_confirm_cnt > 2) {
                                    adj_cnt++;
                                    vel_f = -0.1;
                                    auto txt = std_msgs::msg::String();
                                    txt.data = "too close, move away";
                                    navi_pub->publish(txt);
                                } else if (far_confirm_cnt > 2) {
                                    adj_cnt++;
                                    vel_f = 0.1;
                                    auto txt = std_msgs::msg::String();
                                    txt.data = "too far, move close";
                                    navi_pub->publish(txt);
                                }
                                if (adj_cnt > 5) {
                                    adj_cnt = 0;
                                    far_confirm_cnt = 0;
                                    close_confirm_cnt = 0;
                                }

                                float vx, vy;
                                if (detected_structs.hori_vy < 0) {
                                    vx = -detected_structs.hori_vx;
                                    vy = -detected_structs.hori_vy;
                                } else {
                                    vx = detected_structs.hori_vx;
                                    vy = detected_structs.hori_vy;
                                }
                                float angle_y_hori = acosf(vy);
                                if (angle_y_hori > 0.15) align_confirm_cnt++; else align_confirm_cnt = 0;
                                if (align_confirm_cnt > 2 && yaw_adj_cd == 0) {
                                    yaw_adj_cd = 10;
                                    align_confirm_cnt = 0;
                                    //printf("angle_y_hori %f %f\n", angle_y_hori, vx);
                                    if (vx > 0) tgt_yaw = cur_yaw + angle_y_hori; else tgt_yaw = cur_yaw - angle_y_hori;
                                    auto txt = std_msgs::msg::String();
                                    txt.data = "adjust heading " + (vx > 0 ? std::string("cw ") : std::string("ccw ")) + std::to_string(angle_y_hori * 180 / M_PI) + " from " + std::to_string(cur_yaw * 180 / M_PI) + " to " + std::to_string(tgt_yaw * 180 / M_PI);
                                    navi_pub->publish(txt);
                                }
                            }
                            if (yaw_adj_cd > 0) type_mask = 0x9c7;
                            gettimeofday(&tv, NULL);
                            mavlink_msg_set_position_target_local_ned_pack(mav_sysid, MY_COMP_ID, &msg, tv.tv_sec*1000+tv.tv_usec*0.001, 0, 0, MAV_FRAME_BODY_OFFSET_NED, type_mask, 0, 0, 0, vel_f, vel_r, vel_d, 0, 0, 0, tgt_yaw, 0);
                            len = mavlink_msg_to_send_buffer(buf, &msg);
                            write(uart_fd, buf, len);
                        } else if (move_status == MOVE_UP || move_status == MOVE_DOWN) {
                            float vel_f = 0;
                            float vel_r = 0;
                            float vel_d = 0.2;
                            if (move_status == MOVE_UP) vel_d = -0.2;
                            if (detected_structs.vert_x == 0) {
                            } else {
                                float t = -detected_structs.vert_z / detected_structs.vert_vz;
                                float x = detected_structs.vert_x + detected_structs.vert_vx * t;
                                float y = detected_structs.vert_y + detected_structs.vert_vy * t;
                                if (x < CLOSE_DIST_M) close_confirm_cnt++; else close_confirm_cnt = 0;
                                if (x > FAR_DIST_M) far_confirm_cnt++; else far_confirm_cnt = 0;
                                if (y > 0.2) left_confirm_cnt++; else left_confirm_cnt = 0;
                                if (y< -0.2) right_confirm_cnt++; else right_confirm_cnt = 0;
                                if (close_confirm_cnt > 2) {
                                    adj_cnt++;
                                    vel_f = -0.1;
                                    auto txt = std_msgs::msg::String();
                                    txt.data = "too close, move away";
                                    navi_pub->publish(txt);
                                } else if (far_confirm_cnt > 2) {
                                    adj_cnt++;
                                    vel_f = 0.1;
                                    auto txt = std_msgs::msg::String();
                                    txt.data = "too far, move close";
                                    navi_pub->publish(txt);
                                }
                                if (left_confirm_cnt > 2) {
                                    vel_r = -0.1;
                                    auto txt = std_msgs::msg::String();
                                    txt.data = "too right, move left";
                                    navi_pub->publish(txt);
                                } else if (right_confirm_cnt > 2) {
                                    vel_r = 0.1;
                                    auto txt = std_msgs::msg::String();
                                    txt.data = "too left, move right";
                                    navi_pub->publish(txt);
                                }
                                if (adj_cnt > 5) {
                                    adj_cnt = 0;
                                    far_confirm_cnt = 0;
                                    close_confirm_cnt = 0;
                                }
                            }
                            gettimeofday(&tv, NULL);
                            mavlink_msg_set_position_target_local_ned_pack(mav_sysid, MY_COMP_ID, &msg, tv.tv_sec*1000+tv.tv_usec*0.001, 0, 0, MAV_FRAME_BODY_OFFSET_NED, 0xdc7, 0, 0, 0, vel_f, vel_r, vel_d, 0, 0, 0, 0, 0);
                            len = mavlink_msg_to_send_buffer(buf, &msg);
                            write(uart_fd, buf, len);
                        } else if (move_status == LAND) {
                            mavlink_msg_set_mode_pack(mav_sysid, MY_COMP_ID, &msg, mav_sysid, 1, 9); //land
                            len = mavlink_msg_to_send_buffer(buf, &msg);
                            write(uart_fd, buf, len);
                        }
                    }
                    if (detected_structs.vert_x != 0) {
                        last_detected_structs.vert_x = detected_structs.vert_x;
                        last_detected_structs.vert_y = detected_structs.vert_y;
                        last_detected_structs.vert_z = detected_structs.vert_z;
                        last_detected_structs.vert_vx = detected_structs.vert_vx;
                        last_detected_structs.vert_vy = detected_structs.vert_vy;
                        last_detected_structs.vert_vz = detected_structs.vert_vz;
                    }
                    if (detected_structs.hori_x != 0) {
                        last_detected_structs.hori_x = detected_structs.hori_x;
                        last_detected_structs.hori_y = detected_structs.hori_y;
                        last_detected_structs.hori_z = detected_structs.hori_z;
                        last_detected_structs.hori_vx = detected_structs.hori_vx;
                        last_detected_structs.hori_vy = detected_structs.hori_vy;
                        last_detected_structs.hori_vz = detected_structs.hori_vz;
                    }
                }
            }
        }
    }

    close(uart_fd);
    close(ipc_fd);
    close(ipc_fd2);
    unlink(SERVER_PATH);
    unlink(SERVER_PATH2);

    rclcpp::shutdown();

    printf("bye\n");

    return 0;
}
