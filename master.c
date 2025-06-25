
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <fcntl.h>
#include <unistd.h>
#include <errno.h>
#include <poll.h>

#define BUFFER_MAX 256
#define PINS_MASTER "17, 27, 22, 23, 24" // D0, D1, D2, S_CLK, M_CTRL

// init 함수는 동일
static int gpio_master_init(const char* pin_config) {
    char init_path[] = "/sys/class/gpiocom/master/init";
    int fd = open(init_path, O_WRONLY);
    if (fd == -1) {
        perror("Failed to open master/init");
        return -1;
    }
    write(fd, pin_config, strlen(pin_config));
    close(fd);
    printf("Master initialized with pins: %s\n", pin_config);
    return 0;
}

int main() {
    int data_fd;
    struct pollfd pfd;

    // 1. Master 드라이버 초기화
    if (gpio_master_init(PINS_MASTER) != 0) {
        return 1;
    }

    // 2. Slave가 연결되고 드라이버 내부에서 핸드셰이크 및 속도 보정이 완료되면
    //    문자 디바이스가 I/O 가능 상태가 된다고 가정.
    printf("Waiting for connection and calibration to complete...\n");
    // 실제 구현에서는 드라이버가 sysfs에 상태 파일을 만들어주면 그것을 폴링할 수 있다.
    // 예: /sys/class/gpiocom/master/status 파일 내용이 "ready"가 될 때까지 대기
    sleep(10); // 여기서는 충분한 대기 시간으로 대체

    // 3. 데이터 통신을 위한 문자 디바이스 열기
    data_fd = open("/dev/gpiocom_master", O_RDWR);
    if (data_fd < 0) {
        perror("Failed to open /dev/gpiocom_master");
        return 1;
    }
    printf("Device /dev/gpiocom_master opened. Ready for communication.\n");

    pfd.fd = data_fd;
    pfd.events = POLLIN; // 읽을 데이터가 있는지 감시

    while (1) {
        printf("Waiting for a request from slave...\n");
        int ret = poll(&pfd, 1, -1); // 데이터가 들어올 때까지 무한 대기

        if (ret > 0 && (pfd.revents & POLLIN)) {
            char buffer[BUFFER_MAX] = {0};
            ssize_t len = read(data_fd, buffer, BUFFER_MAX - 1);
            if (len > 0) {
                printf("Received request from slave: %s\n", buffer);

                // 요청 처리 (예: "GET_TEMP" 요청에 온도 값 응답)
                const char* response = "RESPONSE_OK_25C";
                printf("Sending response: %s\n", response);
                write(data_fd, response, strlen(response));
            } else {
                printf("Connection closed or error.\n");
                break;
            }
        }
    }

    close(data_fd);
    return 0;
}