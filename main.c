#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>
#include <unistd.h>
#include <termios.h>
#include <errno.h>
#include "mavlink/common/mavlink.h"

int main(){
    int serial_fd = 0;
    char *uart_name = "/dev/ttyACM0"; // 串口设备文件名，需要根据实际情况修改
    int baudrate = 115200; // 波特率，需要根据实际情况修改

    mavlink_status_t status;
    uint8_t chan = MAVLINK_COMM_0;
    // 打开串口
    serial_fd = open(uart_name, O_RDWR | O_NOCTTY | O_NDELAY);
    if (serial_fd == -1)
    {
        fprintf(stderr, "无法打开串口设备: %s\n", uart_name);
        return EXIT_FAILURE;
    }

    // 配置串口属性
    struct termios uart_config;
    tcgetattr(serial_fd, &uart_config);
    uart_config.c_cflag |= CLOCAL | CREAD;
    uart_config.c_cflag &= ~CSIZE;
    uart_config.c_cflag |= CS8;
    uart_config.c_cflag &= ~PARENB;
    uart_config.c_cflag &= ~CSTOPB;
    uart_config.c_cflag &= ~CRTSCTS;
    uart_config.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
    uart_config.c_iflag &= ~(IXON | IXOFF | IXANY);
    uart_config.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL);
    uart_config.c_oflag &= ~OPOST;
    uart_config.c_cc[VMIN] = 0;
    uart_config.c_cc[VTIME] = 5;
    cfsetispeed(&uart_config, baudrate);
    cfsetospeed(&uart_config, baudrate);
    tcsetattr(serial_fd, TCSANOW, &uart_config);

    // 设置MAVLink协议版本和系统ID
    uint8_t system_id = 1;
    uint8_t component_id = MAV_COMP_ID_ALL;
    uint8_t mavlink_version = MAVLINK_VERSION;

    // 创建MAVLink消息缓冲区
    uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
    uint8_t rx_buf[MAVLINK_MAX_PACKET_LEN];
    // 创建MAVLink消息
    mavlink_message_t msg;
    mavlink_msg_heartbeat_pack(system_id, component_id, &msg, MAV_TYPE_QUADROTOR, MAV_AUTOPILOT_GENERIC, MAV_MODE_PREFLIGHT, mavlink_version, MAV_STATE_STANDBY);

    // 将MAVLink消息序列化到缓冲区
    uint16_t len = mavlink_msg_to_send_buffer(buffer, &msg);

    // 发送缓冲区中的数据到串口
    ssize_t bytes_sent = write(serial_fd, buffer, len);

    if (bytes_sent == -1)
    {
        fprintf(stderr, "写入串口时出现错误: %s\n", strerror(errno));
        return EXIT_FAILURE;
    }
    while(1){
        int n = read(serial_fd, rx_buf, sizeof(rx_buf));
        for (int i = 0; i < n; ++i) {
            if (mavlink_parse_char(chan, rx_buf[i], &msg, &status)){
                    // Handle the message type that you're interested in:
                switch (msg.msgid) {
                    // case MAVLINK_MSG_ID_HEARTBEAT: {
                    //     mavlink_heartbeat_t heartbeat;
                    //     mavlink_msg_heartbeat_decode(&msg, &heartbeat);
                    //     // Handle the heartbeat message...
                    //     printf("1 ");
                    //     break;
                    // }
                    case MAVLINK_MSG_ID_ATTITUDE: {
                        mavlink_attitude_t attitude;
                        mavlink_msg_attitude_decode(&msg, &attitude);
                        //printf("row pitch yaw: %f\t %f\t %f\n",attitude.roll, attitude.pitch, attitude.yaw);
                        // Handle the attitude message...
                        break;
                    }
                    case MAVLINK_MSG_ID_HIGHRES_IMU: {
                        mavlink_highres_imu_t imu;
                        mavlink_msg_highres_imu_decode(&msg, &imu);
                        printf("xacc yacc zacc: %d: %f\t %f\t %f\n",imu.time_usec, imu.xacc, imu.yacc, imu.zacc);
                    }
                    default :
                        break;
                    // Add more cases for other message types that you're interested in...
                }
            }
        }
        //usleep(20000);
    }
}