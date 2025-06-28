/**
 * @file chat.c
 * @brief gpio_comm_drv를 이용한 턴제 채팅 프로그램
 * @version 1.0
 *
 * @note
 * - 이 프로그램은 sender와 listener의 기능을 합칩니다.
 * - 한 번 메시지를 보내고, 한 번 답장을 기다리는 방식으로 동작합니다.
 * - 프로그램 실행 시 받는 인자(argv)를 통해 자신을 식별합니다.
 * - 예: ./chat 0
 *
 * - 컴파일: gcc -Wall -o chat chat.c
 * - 실행 전 확인:
 * 1. `sudo insmod gpio_comm_drv.ko`  // lsmod | grep gpio_comm_drv로 확인
 * - 실행: `sudo ./chat`
 */

 #include <stdio.h>
 #include <stdlib.h>
 #include <string.h>
 #include <fcntl.h>   // open()
 #include <unistd.h>  // read(), write(), close()
 #include <errno.h>   // errno
 
 // 사용할 디바이스 파일 경로. 드라이버에서 export로 생성한 이름과 일치해야 함.
 #define DEVICE_PATH_0 "/dev/gpio_chat_0"
 #define DEVICE_PATH_1 "/dev/gpio_chat_1"
 
 // 읽기 버퍼의 크기. 드라이버의 RX_BUFFER_SIZE와 유사하게 설정.
 #define BUFFER_SIZE 1024
 
 int main(int argc, char *argv[]) {
     int fd; // 파일 디스크립터
     char buffer[BUFFER_SIZE];
     ssize_t bytes_read;
     ssize_t bytes_written;
     
    // 1. 커맨드 라인 인자 확인
    if (argc != 2) {
        // 프로그램 사용법을 잘못 입력했을 경우 안내 메시지를 출력합니다.
        fprintf(stderr, "Usage: %s <your_id>\n", argv[0]);
        fprintf(stderr, "Example: %s 0\n", argv[0]);
        return EXIT_FAILURE;
    }

    // 간단하게 유저는 0 또는 1
    if (argv[1][0] == '0') {
        // 디바이스 생성
        system("sudo sh -c 'echo \"gpio_chat_0,0,18,23,25,16,20\" > /sys/class/gpio_comm/export'");
        printf("Starting GPIO chat as user 0 on %s...\n", DEVICE_PATH_0);
    
        // 디바이스 파일 열기 RW
        fd = open(DEVICE_PATH_0, O_RDWR);
    }
    else {
        // 디바이스 생성
        system("sudo sh -c 'echo \"gpio_chat_1,1,17,22,9,19,26\" > /sys/class/gpio_comm/export'");
        printf("Starting GPIO chat as user 1 on %s...\n", DEVICE_PATH_1);
    
        // 디바이스 파일 열기 RW
        fd = open(DEVICE_PATH_1, O_RDWR);
    }

    if (fd < 0) {
        // 파일 열기 실패 시, 에러 메시지를 출력하고 종료합니다.
        // `perror`는 현재 `errno` 값에 해당하는 시스템 에러 메시지를 출력합니다.
        // (예: "No such file or directory", "Permission denied")
        perror("Failed to open the device");
        fprintf(stderr, "Please check if the driver is loaded and the device node is created correctly.\n");
        return EXIT_FAILURE;
    }

    printf("Device opened successfully. Type a message and press Enter to send.\n");
    printf("Type 'exit' to quit.\n");
    printf("This is a turn-based chat. Please wait for a reply after sending.\n\n");
 

     // 2. 무한 루프를 돌며 전송과 수신 반복
     while (1) {
        ////////////
        // 쓰기
        ////////////

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

         

        ////////////
        // 읽기
        ////////////

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
     printf("Chat stopped.\n");


     // 4. 디바이스 파일 삭제
    // 간단하게 유저는 0 또는 1
    if (argv[1][0] == '0') {
        system("sudo sh -c 'echo \"gpio_chat_0\" > /sys/class/gpio_comm/unexport'");
    }
    else {
        system("sudo sh -c 'echo \"gpio_chat_1\" > /sys/class/gpio_comm/unexport'");
    }

    return EXIT_SUCCESS;
}
 