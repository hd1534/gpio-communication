v3.0

high로 대기할 필요가 없는듯?

그냥 rw모드 대신에 자신이 통신을 원할시 사용할 핀을 0~3번으로 받고, (-1이면 read only)

자동할당 하지말고 그냥 수동 할당 ㄱㄱ

그러고 ack기반으로 갈거니 상대가 접속해있는지 확인 ㄴㄴ

그러면 datapin irq가 없어도 되고 간단해짐
