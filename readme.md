### gpoi 통신 프로토콜 v1.0

GPIO 통신 프로토콜 설계 명세서

#### 1. 개요

본 문서는 두 대의 라즈베리 파이(Raspberry Pi) 간의 안정적인 데이터 통신을 위해 5개의 GPIO 핀을 활용하는 통신 프로토콜을 정의한다. 본 프로토콜은 하나의 마스터(Master)와 하나의 슬레이브(Slave)로 구성된 마스터-슬레이브(Master-Slave) 구조를 채택한다.

주요 특징은 다음과 같다.

- 반이중(Half-duplex) 통신: 하나의 제어 핀을 통해 데이터 전송 방향을 동적으로 전환하여 양방향 통신을 구현한다.
- 동기식 병렬 통신: 클럭(Clock) 신호에 맞춰 3-bit 데이터를 병렬로 전송하여 효율을 높인다.
- 동적 클럭 속도 보정: 통신 시작 시 핸드셰이크(Handshake) 과정을 통해 두 장치 간 최적의 통신 속도를 자동으로 설정한다.
- 패킷 기반 전송 및 오류 검출: 모든 데이터는 표준화된 패킷(Packet) 단위로 전송되며, CRC-16을 통해 데이터 무결성을 보장한다. 오류 발생 시 ACK/NACK 기반의 재전송 메커니즘을 통해 신뢰성을 확보한다.

#### 2. 물리적 연결 및 핀 역할 정의

총 5개의 GPIO 핀을 사용하며, 각 핀의 역할과 입/출력 방향은 통신 모드에 따라 동적으로 변경된다.

| 핀 이름 | gpio bcm |     주 역할      |   Slave -> Master 모드    |   Master -> Slave 모드    |                 비고                  |
| :-----: | :------: | :--------------: | :-----------------------: | :-----------------------: | :-----------------------------------: |
|   D0    | (예: 17) |     데이터 0     | IN (Master) / OUT (Slave) | OUT (Master) / IN (Slave) |        3-bit 병렬 데이터 버스         |
|   D1    | (예: 27) |     데이터 1     | IN (Master) / OUT (Slave) | OUT (Master) / IN (Slave) |                                       |
|   D2    | (예: 22) |     데이터 2     | IN (Master) / OUT (Slave) | OUT (Master) / IN (Slave) |                                       |
|  S_CLK  | (예: 23) |  슬레이브 클럭   | IN (Master) / OUT (Slave) | OUT (Master) / IN (Slave) | Slave가 데이터 전송 시 Clock으로 사용 |
| M_CTRL  | (예: 24) | 마스터 제어/클럭 | OUT (Master) / IN (Slave) | OUT (Master) / IN (Slave) | 모드 제어 및 Master의 Clock으로 사용  |

#### 3. 통신 프로토콜 상세

##### 3.1. 통신 모드 전환

데이터 전송 방향은 마스터 제어 핀(M_CTRL)과 슬레이브 클럭 핀(S_CLK)의 상태에 따라 결정된다.
전환전 항상 D0~D2가 low인걸 확인하고 들어간다.

- Slave-to-Master 모드 (기본 상태):

- 마스터는 M_CTRL 핀을 High로 설정하여 슬레이브에게 전송 권한을 부여한다.
- 마스터의 D0-D2, S_CLK 핀은 IN 모드로 대기한다.
- 슬레이브는 S_CLK를 클럭으로 사용하여 D0-D2 핀으로 데이터를 전송한다.

- Master-to-Slave 모드 전환:
  - 슬레이브가 데이터 전송을 완료하면, S_CLK 핀의 클럭 생성을 중단하고 상태를 Low로 유지한다.
  - 마스터는 S_CLK 핀이 일정 시간(Timeout, 예: 100ms) 이상 Low 상태로 유지되는 것을 감지한다.
  - 이를 '전송 방향 전환' 신호로 간주하고, 마스터는 M_CTRL 핀을 클럭으로 사용하여 데이터 전송을 시작한다. 모든 데이터 전송은 rising edge에 동기화된다.

##### 3.2. 초기 연결 및 클럭 속도 보정

응용 프로그램이 디바이스 드라이버를 open() 할 때, 안정적인 통신을 위해 최적의 클럭 속도를 자동으로 탐색하는 절차를 수행한다.

- 시작: 마스터가 M_CTRL을 High로 설정하여 슬레이브에 신호를 보낸다.
- 속도 테스트: 마스터는 가장 느린 클럭 속도(가장 긴 delay)부터 시작하여, 3-bit 데이터 000(0)부터 111(7)까지 순차적으로 전송한다.
- 슬레이브 응답: 슬레이브는 각 데이터를 정상적으로 수신할 때마다 ACK 신호를 보낸다.
- 속도 증가: 마스터는 ACK를 수신하면 클럭 delay를 점차 줄여(속도를 높여) 2번 과정을 반복한다.
- 최적 속도 결정: 슬레이브로부터 ACK가 오지 않거나 NACK이 수신되는 시점의 바로 이전 클럭 속도를 양측의 최종 통신 속도로 확정한다.

##### 3.3. 데이터 프레임 구조 (Packet Format)

모든 데이터는 128바이트 고정 크기 패킷으로 전송된다.

|           필드            | 크기 (bytes) |                                 설명                                  |
| :-----------------------: | :----------: | :-------------------------------------------------------------------: |
|   SOH (Start of Header)   |      1       |                   0x01 값을 가지는 헤더 시작 플래그                   |
|        Packet Type        |      1       |           패킷 종류 ( 0x10: DATA, 0x20: ACK, 0x30: NACK 등)           |
|    Total Packet Count     |      2       |                 전체 파일 전송에 필요한 총 패킷의 수                  |
|      Sequence Number      |      2       |                  현재 패킷의 순서 번호 (0부터 시작)                   |
|      Payload Length       |      2       |                     실제 데이터의 길이 (0 ~ 118)                      |
|      Payload (Data)       |     118      | 실제 전송 데이터. 길이가 118보다 작으면 나머지는 0으로 채움(Padding). |
|          CRC-16           |      2       |   SOH부터 Payload 끝까지의 데이터를 기반으로 계산된 오류 검출 코드.   |
| EOT (End of Transmission) |      1       |                    0x04 값을 가지는 패킷 끝 플래그                    |
|          총 크기          |     128      |                                                                       |

※ CRC-16 채택 이유: 1바이트의 CRC-8보다 월등한 오류 검출 능력을 제공하여 GPIO 통신에서 발생할 수 있는 노이즈나 신호 왜곡에 효과적으로 대응할 수 있다. 2바이트의 오버헤드는 신뢰성 높은 데이터 전송을 위해 충분히 감수할 가치가 있다.

##### 3.4. 오류 처리 및 복구 메커니즘

ACK/NACK과 타임아웃을 이용하여 신뢰성 있는 데이터 전송을 보장한다.

|     오류 상황      |                        감지 방법                        |                      슬레이브의 대응                       |                                                             마스터의 복구 전략                                                             |
| :----------------: | :-----------------------------------------------------: | :--------------------------------------------------------: | :----------------------------------------------------------------------------------------------------------------------------------------: |
|    데이터 손상     |               수신 데이터의 CRC-16 불일치               |           NACK (Error Code: CRC_ERROR) 패킷 전송           |                                         해당 Sequence Number의 패킷을 재전송한다. (최대 3회 시도)                                          |
| ACK/NACK 응답 유실 | 마스터가 패킷 전송 후 일정 시간(Timeout) 동안 응답 없음 | 슬레이브는 정상적으로 ACK/NACK을 보냈으나 마스터가 못 받음 | 타임아웃 발생 시, 마스터는 동일 패킷을 재전송한다. 슬레이브는 이전에 받은 것과 같은 번호의 패킷이면 중복으로 처리하고 다시 ACK만 전송한다. |
|   완전 통신 두절   |          재전송을 반복해도 계속 타임아웃 발생           |             응답 없음 (하드웨어 연결 문제 등)              |                     3회 재전송 실패 시 통신 실패로 간주. write() 함수는 에러(-EIO)를 반환하여 응용 프로그램에 알린다.                      |

### 1\. GPIO 통신 프로토콜 설계 명세서 (v2.0)

#### 1\. 개요

본 문서는 두 장치(Master, Slave) 간의 5-wire GPIO 통신을 위한 v2.0 프로토콜을 정의한다. 이 프로토콜은 명확한 상태(IDLE, 전송 중, 오류) 정의를 통해 충돌을 방지하고, 동적 속도 보정 및 클라이언트-서버 모델을 통해 안정적이고 효율적인 데이터 교환을 목표로 한다.

#### 2\. 핀 역할 및 상태 정의

- **핀 구성:** D0, D1, D2 (데이터), S_CLK (슬레이브 클럭), M_CTRL (마스터 제어/클럭)
- **기본 상태 신호:**
  - **`IDLE` (유휴/대기):** 제어 핀(M_CTRL, S_CLK)이 `High` 상태. 이는 상대방이 입력을 받을 준비가 되었음을 의미한다.
  - **`TRANSMITTING` (전송 중):** 제어 핀이 클럭킹(`Rising Edge` 동기화). 충돌 방지를 위해 실제 데이터 전송 전 **3회의 예비 클럭킹**을 수행한다.
  - **`ERROR / DISCONNECTED`:** 제어 핀이 `Low` 상태. 통신 오류 또는 연결 종료를 의미한다.

#### 3\. 통신 절차 상세

**3.1. 초기 연결 (Initial Handshake)**

1.  **Master 초기화:**
    - `M_CTRL` 핀을 `OUT`, `High`로 설정한다.
    - `D0-D2`, `S_CLK` 핀을 `IN` 모드로 설정하고 Slave의 연결을 대기한다.
2.  **Slave 초기화:**
    - `M_CTRL` 핀을 `IN`으로 설정한다.
    - `D0-D2`, `S_CLK` 핀을 `OUT`으로 설정한다.
    - `S_CLK`를 `High`로, `D0-D2`를 `Low`로 설정하여 Master에게 연결 준비가 되었음을 알린다.
3.  **연결 인지:**
    - **Master:** `S_CLK` 핀의 `High` 상태를 감지하여 Slave 연결을 인지한다.
    - **Master:** 연결 인지 후, `M_CTRL` 핀을 클럭킹하여 응답한다.
4.  **상호 확인:**
    - **Slave:** `M_CTRL` 핀의 클럭킹을 감지한다.
    - **Slave:** 감지 후, `S_CLK` 핀을 클럭킹하여 최종 응답한다.
5.  **연결 완료:** 양측은 상호 클럭킹을 확인한 후, 각자의 제어 핀(`M_CTRL`, `S_CLK`)을 `High`로 유지한다. 이로써 **IDLE** 상태로 진입하며 속도 보정 단계로 넘어간다.

**3.2. 클럭 속도 보정 (Dynamic Clock Calibration)**

양방향 통신을 위해 Master와 Slave가 각각 송신자가 되어 속도를 보정한다.

- **Phase 1: Slave -\> Master**

  1.  **시작:** 초기 클럭 간격은 100ms로 설정한다.
  2.  **전송:** Slave는 `S_CLK`를 클럭으로 사용하여 3-bit 데이터 `000`(0)부터 `111`(7)까지 순차적으로 Master에게 전송한다.
  3.  **수신 및 응답:** Master는 전송된 숫자를 모두 정확히 수신하면 `ACK` 신호를, 실패하면 `NACK` 신호를 `M_CTRL`을 통해 전송한다.
  4.  **속도 조절:** Slave가 `ACK`를 받으면 클럭 간격을 줄여(속도를 높여) 2번 과정을 반복한다. `NACK`를 받으면, **직전의 성공한 클럭 간격**을 최적 속도(Slave-\>Master)로 확정한다.

- **Phase 2: Master -\> Slave**

  - 위 1\~4번 과정을 역할만 바꾸어 동일하게 진행한다. (Master가 송신자, Slave가 수신자)
  - 이때 Master는 모든 핀을 `OUT`으로, Slave는 모든 핀을 `IN`으로 전환한다.
  - 완료 후, 양방향 최적 클럭 속도가 결정된다.

**3.3. 데이터 전송 (Client-Server Model)**

- **구조:** Slave가 Client (요청), Master가 Server (응답) 역할을 수행한다.

<!-- end list -->

1.  **요청 시작 (Slave):**
    - IDLE 상태(`S_CLK` High)에서 `S_CLK` 핀을 **3회 예비 클럭킹**하여 데이터 전송 시작을 알린다.
    - 이후 클럭에 맞추어 `D0-D2` 핀으로 요청 데이터를 패킷 단위로 전송한다.
2.  **수신 (Master):** Master는 예비 클럭킹을 감지하고, 이후 클럭에 맞추어 데이터를 수신한다.
3.  **전송 완료 (Slave):**
    - 요청 데이터 전송을 마치면, `S_CLK` 핀을 \*\*클럭 간격의 5배 시간 동안 `High`\*\*로 유지하여 전송 종료를 명확히 알린다.
    - 종료 신호 후, Slave는 충돌 방지를 위해 **모든 핀을 `IN` 모드로 전환**한다.
4.  **처리 및 응답 (Master):**
    - Master는 수신된 요청을 처리한다.
    - 처리가 성공하면, `M_CTRL`을 클럭으로 사용하여 응답 데이터를 Slave에게 전송한다.
    - 요청 수신이 실패했다면(CRC 오류 등), `NACK` 신호를 전송한다.

**3.4. 연결 종료 (Termination)**

1.  **종료 요청 (Slave):** IDLE 상태에서, `S_CLK` 핀을 `Low`로 내리고 `D0-D2` 핀을 `High`로 설정한다.
2.  **종료 확인 (Master):** Master가 이 상태를 감지하면, `M_CTRL` 핀을 `Low`로 내려 응답한다.
3.  **종료 완료 (Slave):** Slave가 `M_CTRL`의 `Low` 상태를 감지하면, 모든 핀을 `Low`로 설정한 후 프로세스를 종료한다.

---

위에는 gemini가 정리한 내용, 아래는 내가 브레인 스토밍 한 내용

# 통신 과정?

기본적으로 high상태는 in mode로 되어있다는 신호로 idle상태를 나타낸다.
clocking은 전송중이라는 뜻으로 충돌을 방지하기 위해 3번의 clocking후 전송하기 시작한다.
low상태는 문제가 생기거나 연결이 끊긴것을 의미한다.

## 처음 연결 과정

1. master가 init을 통해 초기화를 진행한다.
2. 1~4번 핀을 in으로 설정하고, 5번핀을 out에 high상태로 대기한다.
3. slave가 init을 통해 초기화를 진행한다.
4. slave가 1~4번 핀을 out으로 설정하고, 5번핀을 in으로 대기한다. 이때 1~3은 low, 4번핀만 high로 설정한다.
5. master가 slave가 붙은걸 인지하면 5번핀을 clocking 한다. (4번핀이 slave에 의해 high인데 clocking하는건 init과정밖에 없다.)
6. slave가 5번핀이 clocking하는걸 인지하면 4번핀을 clocking한다.
7. 서로 연결이 확인되면 master는 5번핀을 high로, slave는 4번핀을 high로두고 속도 조절 과정에 들어간다.

## 처음 연결후 속도 조절 과정

0. 초기 clock 간격을 100ms에서 시작한다.
1. slave가 4번핀을 clocking하면서 1~3번핀을 통해 0~2^3까지 의 숫자를 순서대로 전송하고, 완료후 4번핀을 high로 설정한다.
2. master에서 4번핀이 high가 되면 1~3번핀을 통해 수신된 숫자들을 확인하고, 성공시 slave에게 ACK 신호를 보낸다.
3. slave는 ACK 신호를 받으면 clock의 간격을 줄여서 1번부터 다시 반복한다.
4. clock이 너무 빨라 master에서 제대로 받지 못하면, nack를 보내고, slave는 직전의 clock간격을 최적으로 간주한다.

5. 위 1~4번 과정을 master와 slave를 바꿔 진행한다. (이때 slave는 1~5번핀을 in으로, master는 1~5번핀을 out으로 설정한다.)
6. 적당한 clock 속도를 찾으면 master는 5번핀을 high로, slave는 4번핀을 high로 해 입력 대기상태에 들어간다.

## 데이터 전송 과정

기본적으로 slave가 요청하면 이에 master가 응답하는 client-server구조이다.

1. slave에서 4번핀을 통해 3번의 clocking후 clock에 맞추어 요청사항을 전송한다.
2. master는 위 clock에 맞추어 요청사항을 수신한다.
3. slave가 데이터를 다 전송하면 high로 clock간격의 5배를 유지한후 모든 핀을 in으로 전환한다. (이때 모든핀이 low가 된다.)
4. master는 수신된 데이터를 확인하고, 성공시 slave가 요청한 데이터를 전송하고, 실패시 nack을 전송한다.

## 종료 과정

1. 대기상태에서 slave가 4번핀을 low상태를 유지한체 1~3번핀은 high로 둔다.
2. master가 이를 감지하면 5번핀을 low로 전환한다.
3. slave가 이를 감지하면 1~3번핀을 low로 전환후 종료한다.

# 앱 코드?

```master.c

/**
 * @brief gpio_comm driver를 통한 master 초기화. master가 초기화된후 대기상태로 진입한다. 만약 slave가 연결되면 `/sys/class/gpio_comm/master/connected_slave` 파일이 생성된다.
 */
static int gpio_master_init(
   int pin_d0,
   int pin_d1,
   int pin_d2,
   int pin_s_clk,
   int pin_m_clk,
) {
    char buffer[BUFFER_MAX];
    ssize_t bytes_written;
    int fd;

    fd = open("/sys/class/gpio_comm/master/init", O_WRONLY);
    if (-1 == fd) {
        fprintf(stderr, "Failed to open master/init for writing!\n");
        return (-1);
    }

    bytes_written = snprintf(buffer, BUFFER_MAX, "%d, %d, %d, %d, %d", pin_d0, pin_d1, pin_d2, pin_s_clk, pin_m_clk);
    write(fd, buffer, bytes_written);
    close(fd);

    return (0);
}

int main() {
   int fd;
   int cnt=0;

   // master 초기화
   gpio_master_init();


   // TODO: 파일이 생성될때까지(slave가 연결 될때까지) 대기
   fd = open("/sys/class/gpio_comm/master/connected_slave", O_RDONLY | O_NONBLOCK);
   if (fd < 0) {
      perror("open");
      return 1;
   }

   // TODO: clock 속도 보정

   while (1) {  // TODO: slave가 연결되어있는지 확인
      // slave로부터 요청을 받고 처리하고.
   }

   close(fd);
   return 0;
}

```

```slave.c

/**
 * @brief gpio_comm driver를 통한 slave 초기화. slave가 초기화된후 master에 접근을 시도한다. 만약 master가 연결되면 `/sys/class/gpio_comm/slave/connected_master` 파일이 생성된다.
 */
static int gpio_slave_init(
   int pin_d0,
   int pin_d1,
   int pin_d2,
   int pin_s_clk,
   int pin_m_clk,
) {
    char buffer[BUFFER_MAX];
    ssize_t bytes_written;
    int fd;

    fd = open("/sys/class/gpio_comm/slave/init", O_WRONLY);
    if (-1 == fd) {
        fprintf(stderr, "Failed to open slave/init for writing!\n");
        return (-1);
    }

    bytes_written = snprintf(buffer, BUFFER_MAX, "%d, %d, %d, %d, %d", pin_d0, pin_d1, pin_d2, pin_s_clk, pin_m_clk);
    write(fd, buffer, bytes_written);
    close(fd);

    return (0);
}

static send_data();
static receive_data();

int main() {
   gpio_slave_init();

   // TODO: 파일이 생성될때까지(master가 연결 될때까지) 대기
   fd = open("/sys/class/gpio_comm/slave/connected_master", O_RDONLY | O_NONBLOCK);
   if (fd < 0) {
      perror("open");
      return 1;
   }

   // TODO: clock 속도 보정

   while (1) {  // TODO: master가 연결되어있는지 확인
      // master로부터 요청을 받고 처리하고.
   }

   close(fd);
   return 0;
}
```
