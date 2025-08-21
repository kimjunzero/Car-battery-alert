# 🎯 FPGA 기반 배터리 모니터링 시스템

> 간단 소개: FPGA에서 생성한 배터리 잔량 데이터를 I²C EEPROM에 기록하고, Raspberry Pi가 이를 읽어 OLED에 시각화 or  MP3로 알림까지!
> 

---

## 📌 프로젝트 개요

- **프로젝트명:** **FPGA Battery Monitor**
- **목표:**
    1. FPGA에서 배터리 잔량(가상 값) 생성 → I²C EEPROM에 저장
    2. Raspberry Pi에서 EEPROM 데이터 읽기 → OLED 디스플레이에 그래픽 표시
    3. 임계치(예: 20%) 이하면 MP3 알림 재생
    4. RTC로 현재 시간 OLED 디스플레이에 표시

---

## 🔧 핵심 기능 & 데이터 흐름

1. **FPGA 모듈**
    - 🔄 가상 배터리 잔량 생성 (0~100%, 랜덤 또는 패턴)
    - 📶 I²C Master 구현 → EEPROM 페이지 쓰기
    - 🔒 WP 핀 제어 (쓰기 보호 on/off)
2. **EEPROM (24Cxx)**
    - 💾 1바이트 단위로 배터리 잔량 저장
    - ✅ 쓰기 보호 활성화/비활성화
    - ⏱ ACK 폴링으로 쓰기 완료 확인
3. **Raspberry Pi 애플리케이션**
    - 📡 I²C(Bit Banging) 통해 EEPROM 읽기
    - 📊 OLED(SSD1306)로 배터리 바 그래프 + % 출력, 그래픽 출력, battery state 표시
    - 🎵 threshold 이하 시 MP3 알림 (`mpg123` 활용)

---

## 🛠️ HW 구성 요소

| 모듈 | 모델/부품 | 역할 |
| --- | --- | --- |
| FPGA 보드 | Basys3  | UART 배터리 잔량 생성ㆍI²C Master |
| EEPROM | K24C128 | 데이터 저장 (I²C 슬레이브) |
| Raspberry Pi | Pi 4 | 데이터 읽기ㆍ디스플레이ㆍ알림 |
| OLED | SSD1306 128×64 | 그래픽 UI 출력 |
| 스피커 | MP3 | MP3 오디오 알림 |
| RTC I2C | Pi 4 | 현재 시간 출력 |
| LED BAR | LED | 모드에 따라 시각적 표시 |

---

## 🔍 모듈별 설계 요약

### 1️⃣ FPGA I²C Master

- **FSM 단계:** IDLE ▶ START ▶ ADDR ▶ WRITE ▶ ACK ▶ STOP ▶ STANDBY
- **신호 제어:** `driver_scl_low`, `driver_sda_low`로 open-drain 드라이빙
- **클록 분주:** 100 MHz → 100 kHz (PRESCALE)

### 2️⃣ EEPROM 인터페이스

- **페이지 프로그래밍:** 8바이트 단위
- **WP 동적 제어:** 쓰기 전 Low → 쓰기 후 High
- **ACK 폴링:** t‎WH 준수하며 완료 확인

### 3️⃣ Raspberry Pi 애플리케이션

- **언어 & 라이브러리:** C + /dev + IOCTL 명령어
- **OLED UI:** 배터리 아이콘 + 퍼센트 텍스트 + 그래픽(배터리에 따른)
- 기존 리눅스 커널 I2C 드라이버 사용 → 직접 통신 프로토콜 구현 예정
