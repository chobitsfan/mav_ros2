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
#include "sensor_msgs/msg/range.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "sensor_msgs/msg/point_cloud.hpp"

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

int uart_fd;
unsigned char buf[1024];
uint8_t mav_sysid = 0;
bool in_guided = false;

struct __attribute__((packed)) lines_3d {
//where (vx, vy, vz) is a normalized vector collinear to the line and (x0, y0, z0) is a point on the line.
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

void avd_callback(const geometry_msgs::msg::TwistStamped::SharedPtr twist_msg) {
    if (in_guided) {
        struct timeval tv;
        mavlink_message_t msg;
        gettimeofday(&tv, NULL);
        mavlink_msg_set_position_target_local_ned_pack(mav_sysid, MY_COMP_ID, &msg, tv.tv_sec*1000+(uint32_t)(tv.tv_usec*0.001), mav_sysid, 1, MAV_FRAME_BODY_OFFSET_NED, 0xdc7, 0, 0, 0, twist_msg->twist.linear.x, -twist_msg->twist.linear.y, -twist_msg->twist.linear.z, 0, 0, 0, 0, 0);
        unsigned int len = mavlink_msg_to_send_buffer(buf, &msg);
        write(uart_fd, buf, len);
    }
}

int main(int argc, char *argv[]) {
    struct pollfd pfds[MY_NUM_PFDS];
    struct timeval tv;
    int retval;
    unsigned int len;
    ssize_t avail;
    mavlink_status_t status;
    mavlink_message_t msg;
    // Create new termios struc, we call it 'tty' for convention
    struct termios tty;
    struct sockaddr_un ipc_addr, ipc_addr2;
    int ipc_fd, ipc_fd2;
    int parse_error = 0, packet_rx_drop_count = 0;

    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("mavlink_udp");
    //auto navi_pub = node->create_publisher<std_msgs::msg::String>("navi", 1);
    auto avd_dir_sub = node->create_subscription<geometry_msgs::msg::TwistStamped>("avoid_direction", 1, avd_callback);
    //auto pos_pub = node->create_publisher<geometry_msgs::msg::Point>("pose", 1);

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
                                in_guided = true;
                            } else {
                                in_guided = false;
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
                        mavlink_msg_vision_speed_estimate_pack(mav_sysid, MY_COMP_ID, &msg, tv.tv_sec*1000000+tv.tv_usec, pose[7], -pose[8], -pose[9], covar, 0);
                        len = mavlink_msg_to_send_buffer(buf, &msg);
                        write(uart_fd, buf, len);
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
