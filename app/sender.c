/**
 * @file sender.c
 * @brief gpio_comm_drv를 위한 송신 테스트 프로그램
 * @version 1.0
 *
 * @note
 * - 이 프로그램은 지정된 gpio_comm 디바이스를 읽기/쓰기('rw') 모드로 엽니다.
 * - 사용자로부터 문자열을 입력받아 `write()` 시스템 콜을 통해 드라이버로 전송합니다.
 * - 컴파일: gcc -Wall -o sender sender.c
 * - 실행 전 확인:
 * 1. `sudo insmod gpio_comm_drv.ko`  // lsmod | grep gpio_comm_drv로 확인
 * 2. `sudo sh -c 'echo "gpio_sender,rw,17,22,9,19,26" > /sys/class/gpio_comm/export'`
 * - 실행: `sudo ./sender`
 */

 #include <stdio.h>
 #include <stdlib.h>
 #include <string.h>
 #include <fcntl.h>   // open()
 #include <unistd.h>  // write(), close()
 #include <errno.h>   // errno
 
 // 사용할 디바이스 파일 경로. 드라이버에서 export로 생성한 이름과 일치해야 함.
 #define DEVICE_PATH "/dev/gpio_sender"
 
 // 입력 버퍼의 크기
 #define BUFFER_SIZE 1024
 
 int main(void) {
     int fd; // 파일 디스크립터
     char buffer[BUFFER_SIZE];
     ssize_t bytes_written;
 
     printf("Starting GPIO sender on %s...\n", DEVICE_PATH);
 
     // 1. 디바이스 파일 열기
     // O_RDWR: 읽기/쓰기 모드로 엽니다. write만 할 것이라도,
     // 드라이버의 read 로직(다른 노드의 요청 감지)과 상호작용하기 위해 RDWR이 필요할 수 있습니다.
     fd = open(DEVICE_PATH, O_RDWR);
     if (fd < 0) {
         perror("Failed to open the device");
         fprintf(stderr, "Please check if the driver is loaded and the device node is created correctly.\n");
         return EXIT_FAILURE;
     }
 
     printf("Device opened successfully. Type a message and press Enter to send.\n");
     printf("Type 'exit' to quit.\n");
 
 
     // 2. 무한 루프를 돌며 사용자 입력 받아 전송
     while (1) {
         printf(">> ");
         
         // 사용자로부터 한 줄을 입력받습니다. `fgets`는 버퍼 오버플로우에 안전합니다.
         if (fgets(buffer, BUFFER_SIZE, stdin) == NULL) {
             // Ctrl+D (EOF) 입력 시 루프 종료
             printf("\nEOF received. Exiting.\n");
             break;
         }
 
         // fgets로 읽은 문자열 끝에는 개행 문자(\n)가 포함되므로 제거합니다.
         buffer[strcspn(buffer, "\n")] = '\0';
         
         // "exit"를 입력하면 프로그램 종료
         if (strcmp(buffer, "exit") == 0) {
             printf("Exiting program.\n");
             break;
         }
         
         // 입력된 문자열의 길이가 0이면 전송하지 않음
         if (strlen(buffer) == 0) {
             continue;
         }
 
         // 3. `write()` 시스템 콜을 통해 드라이버로 데이터 전송
         // 드라이버의 write 함수가 버스 점유, 데이터 패킷화, 전송을 모두 처리합니다.
         bytes_written = write(fd, buffer, strlen(buffer));
 
         if (bytes_written < 0) {
             // 쓰기 실패 시 에러 메시지 출력
             perror("Failed to write to the device");
         } else {
             // 성공적으로 전송 완료
             printf("Successfully sent %ld bytes: '%s'\n", bytes_written, buffer);
         }
     }
 
     // 4. 디바이스 파일 닫기
     close(fd);
     printf("Sender stopped.\n");
 
     return EXIT_SUCCESS;
 }
 