/**
 * @file listener.c
 * @brief gpio_comm_drv를 위한 수신 전용 테스트 프로그램
 * @version 1.0
 *
 * @note
 * - 이 프로그램은 지정된 gpio_comm 디바이스를 읽기 전용('r') 모드로 엽니다.
 * - 드라이버로부터 완전한 데이터 패킷이 수신될 때까지 대기(block)합니다.
 * - 수신된 데이터를 표준 출력(stdout)에 출력합니다.
 * - 컴파일: gcc -Wall -o listener listener.c
 * - 실행 전 확인:
 * 1. `sudo insmod gpio_comm_drv.ko`  // lsmod | grep gpio_comm_drv로 확인
 * 2. `sudo sh -c 'echo "gpio_listener,r,18,23,25,16,20" > /sys/class/gpio_comm/export'`
 * - 실행: `sudo ./listener`
 */

 #include <stdio.h>
 #include <stdlib.h>
 #include <string.h>
 #include <fcntl.h>   // open()
 #include <unistd.h>  // read(), close()
 #include <errno.h>   // errno
 
 // 사용할 디바이스 파일 경로. 드라이버에서 export로 생성한 이름과 일치해야 함.
 #define DEVICE_PATH "/dev/gpio_listener"
 
 // 읽기 버퍼의 크기. 드라이버의 RX_BUFFER_SIZE와 유사하게 설정.
 #define BUFFER_SIZE 1024
 
 int main(void) {
     int fd; // 파일 디스크립터
     char buffer[BUFFER_SIZE];
     ssize_t bytes_read;
     
     // 0. 디바이스 생성
     system("sudo sh -c 'echo \"gpio_listener,r,18,23,25,16,20\" > /sys/class/gpio_comm/export'");
 
     printf("Starting GPIO listener on %s...\n", DEVICE_PATH);
 
     // 1. 디바이스 파일 열기
     // O_RDONLY: 읽기 전용 모드로 엽니다.
     fd = open(DEVICE_PATH, O_RDONLY);
     if (fd < 0) {
         // 파일 열기 실패 시, 에러 메시지를 출력하고 종료합니다.
         // `perror`는 현재 `errno` 값에 해당하는 시스템 에러 메시지를 출력합니다.
         // (예: "No such file or directory", "Permission denied")
         perror("Failed to open the device");
         fprintf(stderr, "Please check if the driver is loaded and the device node is created correctly.\n");
         return EXIT_FAILURE;
     }
 
     printf("Device opened successfully. Waiting for data...\n");
 
     // 2. 무한 루프를 돌며 데이터 수신 대기
     while (1) {
         // read() 시스템 콜을 호출하여 데이터를 읽습니다.
         // 드라이버의 read 함수는 완전한 패킷이 수신될 때까지 이 프로세스를 재웁니다(block).
         bytes_read = read(fd, buffer, BUFFER_SIZE - 1);
 
         if (bytes_read < 0) {
             // read() 중 에러가 발생한 경우.
             perror("Failed to read from the device");
             break; // 루프를 종료합니다.
         } else if (bytes_read == 0) {
             // 일반적으로 파일의 끝(EOF)을 의미하지만,
             // 디바이스 파일에서는 거의 발생하지 않습니다.
             printf("End of file reached. Exiting.\n");
             break;
         } else {
             // 성공적으로 데이터를 읽은 경우.
             // C 문자열로 처리하기 위해 마지막에 NULL 문자를 추가합니다.
             buffer[bytes_read] = '\0';
             printf("Received (%ld bytes): %s\n", bytes_read, buffer);
         }
     }
 
     // 3. 디바이스 파일 닫기
     close(fd);
     printf("Listener stopped.\n");
 
     return EXIT_SUCCESS;
 }
 