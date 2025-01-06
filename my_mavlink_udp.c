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
    int parse_error = 0, packet_rx_drop_count = 0;
    bool att_rcved = false;
    float cur_pitch = 0;
    int missions[] = {
        MOVE_LEFT,
        MOVE_UP,
        MOVE_RIGHT,
        MOVE_UP,
        MOVE_LEFT,
        LAND,
    };
    int mission_idx = -1;
    float cur_vio_x = 0, cur_vio_y = 0, cur_vio_z = 0;
    float wp_vio_x = 0, wp_vio_y = 0, wp_vio_z = 0;
    struct lines_3d detected_structs;
    int confirm_cnt = 0;
    int navi_status = SEARCH_STRUCT_CROSS;
    int move_status = HOVER;

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
                                    printf("mission start\n");
                                    mission_idx = 0;
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
                            cur_pitch = att.pitch;
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
                    //float t = -vert_z / vert_vz;
                    //printf("vert_struct %f %f\n", vert_x + vert_vx * t, vert_y + vert_vy * t);
                    //printf("detected_structs %f %f\n", detected_structs.vert_x, detected_structs.hori_y);
                    if (mission_idx >= 0 && mission_idx < (sizeof(missions) / sizeof(missions[0]))) {
                        if (navi_status == SEARCH_STRUCT_CROSS) {
                            if (detected_structs.vert_x != 0 && detected_structs.hori_x != 0) confirm_cnt++;
                            if (confirm_cnt > 2) {
                                printf("cross detected\n");
                                confirm_cnt = 0;
                                navi_status = PASS_STRUCT_CROSS;
                                move_status = missions[mission_idx];
                            }
                        } else if (navi_status == PASS_STRUCT_CROSS) {
                            if (detected_structs.vert_x == 0 || detected_structs.hori_x == 0) confirm_cnt++;
                            if (confirm_cnt > 2) {
                                printf("cross passed\n");
                                confirm_cnt = 0;
                                navi_status = SEARCH_STRUCT_CROSS;
                                if (mission_idx >= 0) mission_idx++;
                            }
                        }
                        if (move_status == MOVE_RIGHT || move_status == MOVE_LEFT) {
                            float vel_r = 0.2;
                            if (move_status == MOVE_LEFT) vel_r = -0.2;
                            gettimeofday(&tv, NULL);
                            mavlink_msg_set_position_target_local_ned_pack(mav_sysid, MY_COMP_ID, &msg, tv.tv_sec*1000+tv.tv_usec*0.001, 0, 0, MAV_FRAME_BODY_OFFSET_NED, 0xdc7, 0, 0, 0, 0, vel_r, 0, 0, 0, 0, 0, 0);
                            len = mavlink_msg_to_send_buffer(buf, &msg);
                            write(uart_fd, buf, len);
                        } else if (move_status == MOVE_RIGHT || move_status == MOVE_LEFT) {
                            float vel_d = 0.2;
                            if (move_status == MOVE_UP) vel_d = -0.2;
                            gettimeofday(&tv, NULL);
                            mavlink_msg_set_position_target_local_ned_pack(mav_sysid, MY_COMP_ID, &msg, tv.tv_sec*1000+tv.tv_usec*0.001, 0, 0, MAV_FRAME_BODY_OFFSET_NED, 0xdc7, 0, 0, 0, 0, 0, vel_d, 0, 0, 0, 0, 0);
                            len = mavlink_msg_to_send_buffer(buf, &msg);
                            write(uart_fd, buf, len);
                        } else if (move_status == LAND) {
                            mavlink_msg_set_mode_pack(mav_sysid, MY_COMP_ID, &msg, mav_sysid, 1, 9); //land
                            len = mavlink_msg_to_send_buffer(buf, &msg);
                            write(uart_fd, buf, len);
                        }
                    }
                    /*--detect_cd;
                    if (mission_idx >= 0) {
                        bool no_new_cmd = true;
                        if (detect_cd < 0) {
                            int struct_detect = rack[0];
                            int* mission = missions[mission_idx];
                            int struct_expect = mission[0];
                            if (struct_expect & struct_detect ) {
                                float vel_r = 0, vel_d = 0;
                                int cmd = mission[1];
                                if (cmd == 1) {
                                    vel_r = -0.2;
                                } else if (cmd == 2) {
                                    vel_d = -0.2;
                                } else if (cmd == 3) {
                                    vel_r = 0.2;
                                } else if (cmd == 4) {
                                    vel_d = 0.2;
                                }
                                gettimeofday(&tv, NULL);
                                mavlink_msg_set_position_target_local_ned_pack(mav_sysid, MY_COMP_ID, &msg, tv.tv_sec*1000+tv.tv_usec*0.001, 0, 0, MAV_FRAME_BODY_OFFSET_NED, 0xdc7, 0, 0, 0, 0, vel_r, vel_d, 0, 0, 0, 0, 0);
                                len = mavlink_msg_to_send_buffer(buf, &msg);
                                write(uart_fd, buf, len);

                                if (cmd > 0) {
                                    ++mission_idx;
                                    printf("mission idx: %d\n", mission_idx);
                                    wp_vio_x = cur_vio_x;
                                    wp_vio_y = cur_vio_y;
                                    wp_vio_z = cur_vio_z;
                                }
                                detect_cd = 40;
                                no_new_cmd = false;
                            }
                        }
                        if (no_new_cmd && mission_idx > 0) {
                            int dist_mm = rack[1];
                            if (dist_mm == 10000) {
                            } else {
                                float vel_f = 0, vel_r = 0, vel_d = 0;
                                int cmd = missions[mission_idx-1][1];
                                if (cmd == 1) {
                                    vel_r = -0.2;
                                } else if (cmd == 2) {
                                    vel_d = -0.2;
                                } else if (cmd == 3) {
                                    vel_r = 0.2;
                                } else if (cmd == 4) {
                                    vel_d = 0.2;
                                }
                                if (dist_mm < 500) vel_f = -0.1; else if (dist_mm > 900) vel_f = 0.1;
                                gettimeofday(&tv, NULL);
                                mavlink_msg_set_position_target_local_ned_pack(mav_sysid, MY_COMP_ID, &msg, tv.tv_sec*1000+tv.tv_usec*0.001, 0, 0, MAV_FRAME_BODY_OFFSET_NED, 0xdc7, 0, 0, 0, vel_f, vel_r, vel_d, 0, 0, 0, 0, 0);
                                len = mavlink_msg_to_send_buffer(buf, &msg);
                                write(uart_fd, buf, len);
                            }
                        }
                    }*/
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
