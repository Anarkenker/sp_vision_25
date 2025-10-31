#include "send_control.hpp"

#include <netinet/in.h>
#include <arpa/inet.h>
#include <sys/socket.h>
#include <unistd.h>
#include <termios.h>
#include <stdio.h>
#include <cerrno>
#include <cstring>
#include "tools/logger.hpp"

// #include "data_manager/parameter_loader.h"  // 该文件不存在，注释掉

int64_t port_num = 11452;
int sockfd;
sockaddr_in serv_addr;
sockaddr_in addr;

constexpr double RAD2DEG = 57.29577951308232;

enum ROBOT_MODE
{
    ROBOT_NO_FORCE,
    ROBOT_FINISH_INIT,
    ROBOT_FOLLOW_GIMBAL,
    ROBOT_SEARCH,
    ROBOT_IDLE,
    ROBOT_NOT_FOLLOW
};

struct Vison_control
{
    uint8_t header;
    float yaw_set;
    float pitch_set;
    bool fire;

    ROBOT_MODE mode;
} __attribute__((packed));

void init_send(std::string ip)
{
    addr.sin_family = AF_INET;
    addr.sin_addr.s_addr = inet_addr(ip.c_str());
    addr.sin_port = htons(11451);


    sockfd = socket(AF_INET, SOCK_DGRAM, 0);
    if (sockfd < 0) {
        printf("can't open socket\n");
    }

    serv_addr.sin_family = AF_INET;
    serv_addr.sin_addr.s_addr = INADDR_ANY;
    serv_addr.sin_port = htons(port_num);


    if (bind(sockfd, (sockaddr *)&serv_addr, sizeof(serv_addr)) < 0) {
        printf("can't bind socket fd with port number");
    }
}

void send_control(double yaw_set, double pitch_set, bool fire)
{
    Vison_control pkg{};

    pkg.header = 0x6A;
    // 这个或许要改一改， 哨兵变成单头哨兵了。
    pkg.yaw_set = yaw_set;
    pkg.pitch_set = pitch_set;
    pkg.fire = fire;
    pkg.mode = ROBOT_FOLLOW_GIMBAL;


    //printf("send control\n");
    auto n = sendto(
    sockfd,
    (const char *)(&pkg),
    sizeof(pkg),
    MSG_CONFIRM,
    (const struct sockaddr *)&addr,
    sizeof(addr));

    if (n < 0) {
        tools::logger()->warn("[GKDControl] Failed to send control packet: {}", std::strerror(errno));
    } else {
        tools::logger()->info(
            "[GKDControl] Send yaw {:.4f} rad ({:.2f} deg), pitch {:.4f} rad ({:.2f} deg), fire {}",
            yaw_set, yaw_set * RAD2DEG, pitch_set, pitch_set * RAD2DEG, fire ? "true" : "false");
    }
}
