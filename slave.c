
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <fcntl.h>
#include <unistd.h>
#include <errno.h>

#define BUFFER_MAX 256
#define PINS_SLAVE "17, 27, 22, 23, 24" // 핀 번호는 양쪽이 동일하게 설정

// init 함수는 동일
static int gpio_slave_init(const char* pin_config) {
    char init_path[] = "/sys/class/gpiocom/slave/init";
    int fd = open(init_path, O_WRONLY);
    if (fd == -1) {
        perror("Failed to open slave/init");
        return -1;
    }
    write(fd, pin_config, strlen(pin_config));
    close(fd);
    printf("Slave initialized with pins: %s\n", pin_config);
    return 0;
}

int main() {
    int data_fd;

    // 1. Slave 드라이버 초기화
    if (gpio_slave_init(PINS_SLAVE) != 0) {
        return 1;
    }

    // 2. Master와 연결 및 속도 보정 대기
    printf("Trying to connect to master and calibrating...\n");
    sleep(10); // 드라이버가 준비될 때까지 대기

    // 3. 데이터 통신을 위한 문자 디바이스 열기
    data_fd = open("/dev/gpiocom_slave", O_RDWR);
    if (data_fd < 0) {
        perror("Failed to open /dev/gpiocom_slave");
        return 1;
    }
    printf("Device /dev/gpiocom_slave opened. Ready for communication.\n");

    for (int i = 0; i < 5; i++) {
        char request[BUFFER_MAX];
        snprintf(request, BUFFER_MAX, "REQUEST_DATA_%d", i);

        // 4. Master에게 요청 전송
        printf("Sending request: %s\n", request);
        if (write(data_fd, request, strlen(request)) < 0) {
            perror("Failed to write request");
            break;
        }

        // 5. Master로부터 응답 수신
        char response[BUFFER_MAX] = {0};
        printf("Waiting for response...\n");
        ssize_t len = read(data_fd, response, BUFFER_MAX - 1);
        if (len > 0) {
            printf("Received response from master: %s\n\n", response);
        } else {
            printf("Failed to get response or connection closed.\n");
            break;
        }
        sleep(2); // 2초마다 요청
    }

    close(data_fd);
    printf("Communication finished.\n");
    return 0;
}