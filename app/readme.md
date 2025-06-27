# GPIO 통신 드라이버 테스트 프로그램 사용법

이 가이드는 gpio_comm_drv.ko 커널 드라이버를 테스트하기 위한 listener와 sender 프로그램을 컴파일하고 실행하는 방법을 안내합니다.

## 주의사항

실행전

```shell
$ raspi-gpio get 18,23,25,16,20,17,22,9,19,26
```

처럼 모든핀이 input인지 확인후에 선을 연결할것!

### 선 꽂을때 참고

```shell
sudo su
insmod /home/pi/projects/sysprog_material/gpio_drv.ko
cd /sys/class/sysprog_gpio

# gpio export
echo 17 > export
echo 22 > export
echo 9 > export
echo 19 > export
echo 26 > export

# gpio 상태 확인
raspi-gpio get 18,23,25,16,20,17,22,9,19,26

# gpio direction 설정
echo out > gpio17/direction
echo out > gpio22/direction
echo out > gpio9/direction
echo out > gpio19/direction
echo out > gpio26/direction

# gpio value 설정
echo 1 > gpio17/value
echo 1 > gpio22/value
echo 1 > gpio9/value
echo 1 > gpio19/value
echo 1 > gpio26/value

# gpio 상태 확인
raspi-gpio get 18,23,25,16,20,17,22,9,19,26


# gpio value 설정
echo 0 > gpio17/value
echo 0 > gpio22/value
echo 0 > gpio9/value
echo 0 > gpio19/value
echo 0 > gpio26/value

# gpio 상태 확인
raspi-gpio get 18,23,25,16,20,17,22,9,19,26

# gpio direction 설정
echo in > gpio17/direction
echo in > gpio22/direction
echo in > gpio9/direction
echo in > gpio19/direction
echo in > gpio26/direction
```

## 준비물

- GPIO 핀으로 연결된 2대의 라즈베리 파이 (또는 테스트 목적의 1대)
- 리눅스 커널 헤더 (sudo apt install raspberrypi-kernel-headers)
- gpio_comm_drv.c 파일 및 컴파일된 gpio_comm_drv.ko
- listener.c, sender.c 소스 코드 파일

## 1단계: 커널 드라이버 로드

먼저 작성하신 커널 드라이버를 커널에 로드해야 합니다.

```shell
# 드라이버 소스가 있는 곳에서 컴파일
$ make

# 커널 모듈 삽입
$ sudo insmod gpio_comm_drv.ko
```

## 2단계: 통신용 디바이스 노드 생성

sysfs를 통해 각 통신 세션에 대한 디바이스 파일을 생성합니다.
두 장치는 반드시 동일한 GPIO 핀 구성을 사용해야 합니다.

### 터미널 1 (Listener 용):

r (읽기 전용) 모드로 디바이스를 생성합니다.

```shell
# echo "이름,모드,제어핀,데이터0,데이터1,데이터2,데이터3" > /sys/class/gpio_comm/export
$ sudo sh -c 'echo "gpio_listener,r,18,23,25,12,20" > /sys/class/gpio_comm/export'
```

/dev/gpio_listener 파일이 생성됩니다.

### 터미널 2 (Sender 용):

rw (읽기/쓰기) 모드로 디바이스를 생성합니다.

```shell
# echo "이름,모드,제어핀,데이터0,데이터1,데이터2,데이터3" > /sys/class/gpio_comm/export
$ sudo sh -c 'echo "gpio_sender,rw,17,22,9,6,26" > /sys/class/gpio_comm/export'
```

/dev/gpio_sender 파일이 생성됩니다.

## 3단계: 테스트 프로그램 컴파일

아래 내용을 Makefile이라는 이름의 파일로 저장한 뒤, listener.c와 sender.c가 있는 디렉터리에서 make 명령을 실행하세요.

```makefile
# Makefile
CC = gcc
CFLAGS = -Wall -Werror
TARGETS = listener sender

all: $(TARGETS)

listener: listener.c
	$(CC) $(CFLAGS) -o listener listener.c

sender: sender.c
	$(CC) $(CFLAGS) -o sender sender.c

clean:
	rm -f $(TARGETS)
```

컴파일 실행:

```shell
$ make
```

listener와 sender 실행 파일이 생성됩니다.

##4단계: 프로그램 실행

### 터미널 1 (Listener 실행):

```shell
# 디바이스 파일 권한 문제 발생 시 sudo 사용
sudo ./listener
```

이제 이 터미널은 GPIO를 통해 데이터가 들어오기를 무한정 기다립니다.

### 터미널 2 (Sender 실행):

```shell
# 디바이스 파일 권한 문제 발생 시 sudo 사용
sudo ./sender
```

프롬프트가 나타나면 전송할 메시지를 입력하고 Enter 키를 누르세요.
listener 터미널에 해당 메시지가 출력되는 것을 확인할 수 있습니다.

## 5단계: 정리

테스트가 끝나면 unexport를 통해 디바이스 노드를 제거하고 커널 모듈을 언로드합니다.

```shell
sudo sh -c 'echo "gpio_listener" > /sys/class/gpio_comm/unexport'
sudo sh -c 'echo "gpio_sender" > /sys/class/gpio_comm/unexport'
sudo rmmod gpio_comm_drv
```
