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
//#include <ros/ros.h>
//#include <sensor_msgs/Imu.h>
//#include <geometry_msgs/PoseStamped.h>
//#include <nav_msgs/Odometry.h>
#include "mavlink/ardupilotmega/mavlink.h"

#define MY_COMP_ID 191
#define MY_NUM_PFDS 3
#define SERVER_PATH "/tmp/chobits_server"
#define SERVER_PATH2 "/tmp/chobits_server2"

#define RACK_NXT_VERT_MAX_M 2
#define RACK_VERT_STOP_COUNT 1
#define RACK_KEEP_DIST_MM 800
#define RACK_KEEP_DIST_MARGIN_MM 200

void sig_func(int sig) {
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
    int64_t time_offset_us = 0;
    int parse_error = 0, packet_rx_drop_count = 0;
    int64_t tc1_sent = 0;
    bool guided_mode = false;
    int rack_vert_cd = 5;
    int rack_angle_cd = 5;
    bool go_left = true;
    int rack_vert_count = 0;

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

    signal(SIGINT, sig_func);

    printf("hello\n");

    //ros::init(argc, argv, "mavlink_udp");
    //ros::NodeHandle ros_nh;
    //ROS_INFO("mavlink_udp ready");

    //ros::Publisher imu_pub = ros_nh.advertise<sensor_msgs::Imu>("/chobits/imu", 100);
    //ros::Publisher goal_pub = ros_nh.advertise<geometry_msgs::PoseStamped>("/goal", 1);
    //ros::Publisher odo_pub = ros_nh.advertise<nav_msgs::Odometry>("/chobits/odometry", 10);

    memset(&status, 0, sizeof(status));

    while (true) {
        retval = poll(pfds, MY_NUM_PFDS, -1);
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
                            }
                            if (time_offset_us == 0) {
                                gettimeofday(&tv, NULL);
                                tc1_sent = tv.tv_sec*1000000000+tv.tv_usec*1000;
                                mavlink_msg_timesync_pack(mav_sysid, MY_COMP_ID, &msg, 0, tc1_sent, mav_sysid, 1);
                                len = mavlink_msg_to_send_buffer(buf, &msg);
                                write(uart_fd, buf, len);

                                mavlink_msg_system_time_pack(mav_sysid, MY_COMP_ID, &msg, tv.tv_sec*1000000+tv.tv_usec, 0);
                                len = mavlink_msg_to_send_buffer(buf, &msg);
                                write(uart_fd, buf, len);

                                mavlink_msg_set_gps_global_origin_pack(mav_sysid, MY_COMP_ID, &msg, mav_sysid, 247749434, 1210443077, 100000, tv.tv_sec*1000000+tv.tv_usec);
                                len = mavlink_msg_to_send_buffer(buf, &msg);
                                write(uart_fd, buf, len);
                            }
                            if (hb.custom_mode == COPTER_MODE_GUIDED) {
                                guided_mode = true;
                                ++rack_vert_cd;
                                ++rack_angle_cd;
                            } else {
                                guided_mode = false;
                            }
                        } else if (msg.msgid == MAVLINK_MSG_ID_TIMESYNC) {
                            mavlink_timesync_t ts;
                            mavlink_msg_timesync_decode(&msg, &ts);
                            if (ts.ts1 == tc1_sent) {
                                time_offset_us = (ts.ts1 - ts.tc1)/1000;
                                printf("time offset %ld\n", time_offset_us);
                            }
                        } else if (msg.msgid == MAVLINK_MSG_ID_STATUSTEXT) {
                            mavlink_statustext_t txt;
                            mavlink_msg_statustext_decode(&msg, &txt);
                            printf("fc: %s\n", txt.text);
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
                }
            }
            if (pfds[2].revents & POLLIN) {
                int rack[8] = {0};
                if (recv(ipc_fd2, rack, sizeof(rack), 0) > 0) {
                    if (rack[0] == 0) {
                        if (guided_mode && rack_vert_cd > 4) {
                            rack_vert_cd = 0;
                            printf("vertical bar, count %d, dist %d mm\n", rack_vert_count, rack[1]);
                            float f_adj = 0, d_adj = 0;
                            if (rack[1] < (RACK_KEEP_DIST_MM - RACK_KEEP_DIST_MARGIN_MM)) f_adj = -RACK_KEEP_DIST_MARGIN_MM * 0.001; else if (rack[1] > (RACK_KEEP_DIST_MM + RACK_KEEP_DIST_MARGIN_MM)) f_adj = RACK_KEEP_DIST_MARGIN_MM * 0.001;
                            if (rack[2] > 200) d_adj = -0.1; else if (rack[2] < -100) d_adj = 0.2;
                            if (rack_vert_count >= RACK_VERT_STOP_COUNT) {
                                go_left = !go_left;
                                rack_vert_count = 0;
                            }
                            float r_dst = RACK_NXT_VERT_MAX_M;
                            if (go_left) r_dst = -RACK_NXT_VERT_MAX_M;
                            ++rack_vert_count;
                            gettimeofday(&tv, NULL);
                            mavlink_msg_set_position_target_local_ned_pack(mav_sysid, MY_COMP_ID, &msg, tv.tv_sec*1000+tv.tv_usec*0.001, 0, 0, MAV_FRAME_BODY_OFFSET_NED, 0xdf8, f_adj, r_dst, d_adj, 0, 0, 0, 0, 0, 0, 0, 0);
                            len = mavlink_msg_to_send_buffer(buf, &msg);
                            write(uart_fd, buf, len);
                        }
                    } else if (rack[0] == 1) {
                        if (guided_mode && rack_angle_cd > 4) {
                            rack_angle_cd = 0;
                            int dir = -1;
                            float deg = rack[1] * 0.01f;
                            if (deg < 0) {
                                dir = 1;
                                deg = -deg;
                            }
                            mavlink_msg_command_long_pack(mav_sysid, MY_COMP_ID, &msg, mav_sysid, 1, MAV_CMD_CONDITION_YAW, 0, deg, 0, dir, 1, 0, 0, 0);
                            len = mavlink_msg_to_send_buffer(buf, &msg);
                            write(uart_fd, buf, len);
                            printf("yaw %f degree, dir %d\n", deg, dir);
                        }
                    }
                }
            }
        } else break;
    }

    close(uart_fd);
    close(ipc_fd);
    close(ipc_fd2);
    unlink(SERVER_PATH);
    unlink(SERVER_PATH2);

    printf("bye\n");

    return 0;
}
