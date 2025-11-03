# 🚀 SX126X (E22) LoRa HAT 설정 및 사용 가이드

라즈베리파이에서 SX126X LoRa 모듈을 사용하기 위한 완벽 가이드

## 📋 목차
1. [하드웨어 연결](#1-하드웨어-연결)
2. [시스템 설정](#2-시스템-설정)
3. [파일 구성](#3-파일-구성)
4. [사용법](#4-사용법)
5. [문제 해결](#5-문제-해결)

---

## 1. 하드웨어 연결

### 🔌 필수 연결

LoRa HAT를 라즈베리파이에 연결:

```
라즈베리파이 GPIO    →    LoRa HAT
─────────────────────────────────────
Pin 6  (GND)         →    GND
Pin 8  (GPIO 14 TX)  →    RXD
Pin 10 (GPIO 15 RX)  →    TXD
Pin 2  (5V)          →    VCC
```

### 🎛️ 모드 제어 핀 (선택사항)

모드를 **Python으로 제어**하려면 M0, M1 연결 필요:

```
라즈베리파이 GPIO    →    LoRa HAT
─────────────────────────────────────
Pin 11 (GPIO 17)     →    M0
Pin 13 (GPIO 27)     →    M1
```

**또는** 점퍼로 수동 제어:

| M0 | M1 | 모드 | 용도 |
|----|----|------|------|
| 0  | 0  | Normal | 송수신 모드 |
| 0  | 1  | Config | 설정 변경 모드 |
| 1  | 0  | WOR | 저전력 수신 대기 |
| 1  | 1  | Sleep | 절전 모드 |

> 💡 **팁:** M0, M1 연결 없이 사용하려면 점퍼로 Normal(0,0) 고정

---

## 2. 시스템 설정

### 📦 필요 패키지 설치

```bash
# Python 패키지 설치
sudo apt-get update
sudo apt-get install -y python3-pip python3-serial python3-rpi.gpio

# 시리얼 통신 라이브러리
pip3 install pyserial RPi.GPIO
```

### ⚙️ 시리얼 포트 활성화

```bash
# 1. 설정 도구 실행
sudo raspi-config

# 2. 메뉴 선택:
#    3. Interface Options
#    → I6 Serial Port
#    → Login shell: NO (중요!)
#    → Serial interface: YES
#    → Finish & Reboot

# 3. 재부팅
sudo reboot
```

### 🔍 시리얼 포트 확인

```bash
ls -l /dev/tty{AMA,S}*
# 출력 예시:
# /dev/ttyAMA0  ← 주로 사용
# /dev/ttyS0
```

### 👥 사용자 권한 설정

```bash
# dialout 그룹에 사용자 추가
sudo usermod -a -G dialout $USER

# 로그아웃 후 재로그인 필요
```

---

## 3. 파일 구성

```
/home/pi/jm/
├── e22_mode.py          # LoRa 모드 변경 (Normal/Config)
├── e22_config.py        # LoRa 설정 (주소, 채널, 파워 등)
├── e22_trans.py         # 투명 전송 모드 설정
├── check_lora_pid.c     # LoRa 모듈 PID 확인 (C 프로그램)
├── check_lora_pid       # 컴파일된 실행 파일
└── README_LORA.md       # 이 파일
```

### 📂 추가 예제 코드

```
SX126X_LoRa_HAT_Code/
├── raspberrypi/python/
│   ├── main.py          # 송수신 예제
│   └── sx126x.py        # LoRa 드라이버 라이브러리
└── stm32/               # STM32용 C 코드 (참고용)
```

---

## 4. 사용법

### 🎯 빠른 시작 (Quick Start)

```bash
# 1. Config 모드로 전환 (설정 변경용)
sudo python3 e22_mode.py config

# 2. 현재 설정 확인
sudo python3 e22_config.py --verify

# 3. Normal 모드로 전환 (송수신용)
sudo python3 e22_mode.py normal

# 4. 송수신 테스트
cd SX126X_LoRa_HAT_Code/raspberrypi/python
sudo python3 main.py
```

---

### 📡 모드 변경

```bash
# Normal 모드 (송수신)
sudo python3 e22_mode.py normal

# Config 모드 (설정 변경)
sudo python3 e22_mode.py config
```

**사용법:**
```
sudo python3 e22_mode.py [normal|config]
```

**주의:** M0, M1 핀이 연결되어 있어야 작동합니다.

---

### ⚙️ LoRa 모듈 설정

#### 현재 설정 확인

```bash
sudo python3 e22_config.py --verify
```

출력 예시:
```
📊 현재 설정:
  주소     : 0x0000
  네트워크 : 0x00
  UART     : 9600 bps
  패리티   : 8N1
  무선속도 : 2.4k
  채널     : 0x32 (50 → 900.125 MHz)
```

#### 설정 변경

```bash
# 기본 설정
sudo python3 e22_config.py \
  --addr 0x0000 \
  --netid 0x00 \
  --baud 9600 \
  --parity 8N1 \
  --adr 2.4k \
  --channel 0x32 \
  --mode save \
  --verify

# 임시 설정 (전원 꺼지면 초기화됨)
sudo python3 e22_config.py --mode temp --channel 0x20

# 무선으로 설정 (OTA)
sudo python3 e22_config.py --mode wireless --channel 0x15
```

**주요 옵션:**

| 옵션 | 설명 | 가능한 값 |
|------|------|-----------|
| `--addr` | 모듈 주소 | 0x0000 ~ 0xFFFF |
| `--netid` | 네트워크 ID | 0x00 ~ 0xFF |
| `--baud` | UART 속도 | 1200, 2400, 4800, 9600, 19200, 38400, 57600, 115200 |
| `--parity` | 패리티 비트 | 8N1, 8O1, 8E1 |
| `--adr` | 무선 속도 | 0.3k, 1.2k, 2.4k, 4.8k, 9.6k, 19.2k, 38.4k, 62.5k |
| `--channel` | 채널 | 0x00 ~ 0x50 (850.125 ~ 930.125 MHz) |
| `--mode` | 저장 모드 | save (영구), temp (임시), wireless (무선) |
| `--port` | 시리얼 포트 | /dev/ttyAMA0 (기본값) |

**채널 계산:**
```
주파수(MHz) = 850.125 + (채널 번호 × 1.0)

예시:
채널 0x00 = 850.125 MHz
채널 0x17 = 873.125 MHz (기본값)
채널 0x32 = 900.125 MHz
```

---

### 📶 RSSI 신호 강도 측정

#### 실시간 RSSI 모니터링

```bash
# 1초마다 RSSI 값 출력
sudo python3 e22_config.py --rssi-log --interval 1.0

# 출력 예시:
# [2025-11-03T10:30:45] RSSI = -85 dBm
# [2025-11-03T10:30:46] RSSI = -87 dBm
```

#### CSV 파일로 로깅

```bash
# RSSI 값을 CSV 파일로 저장
sudo python3 e22_config.py --rssi-log --csv rssi_data.csv --interval 0.5

# Ctrl+C로 중단
```

---

### 🔄 투명 전송 모드 설정

투명 전송 모드는 **주소/채널 헤더 없이 데이터만 전송**하는 모드입니다.

```bash
# 영구 저장
sudo python3 e22_trans.py --save --verify_after

# 임시 설정
sudo python3 e22_trans.py --verify_after
```

---

### 💻 C 프로그램 사용

#### PID 확인 프로그램

```bash
# 컴파일 (최초 1회)
gcc -o check_lora_pid check_lora_pid.c

# 실행
sudo ./check_lora_pid
```

출력 예시:
```
[*] PID 명령어 전송 완료
[*] PID 응답 수신됨: 03 22 XX XX XX XX XX
[✅] E22 시리즈 LoRa 장치로 확인됨
```

---

### 📨 송수신 예제

#### Python 예제 실행

```bash
cd /home/pi/jm/SX126X_LoRa_HAT_Code/raspberrypi/python
sudo python3 main.py
```

**사용법:**
- `i` 키: 메시지 전송
- `s` 키: 10초마다 CPU 온도 자동 전송
- `c` 키: 자동 전송 중단
- `Esc` 키: 프로그램 종료

**메시지 전송 형식:**
```
0,868,Hello World
↑  ↑   ↑
주소 채널 메시지
```

---

## 5. 문제 해결

### ❌ 시리얼 포트 열기 실패

**증상:**
```
Permission denied: '/dev/ttyAMA0'
```

**해결:**
```bash
# 1. 권한 확인
ls -l /dev/ttyAMA0

# 2. dialout 그룹 추가
sudo usermod -a -G dialout $USER

# 3. 로그아웃 후 재로그인 또는
su - $USER

# 4. 임시 해결 (권장하지 않음)
sudo chmod 666 /dev/ttyAMA0
```

---

### ❌ 모드 변경 실패

**증상:**
```
sudo python3 e22_mode.py config
# 응답 없음 또는 오류
```

**원인:**
- M0, M1 핀이 연결되지 않음
- GPIO 핀 번호가 잘못됨

**해결:**
```bash
# 1. 하드웨어 연결 확인
#    GPIO 17 → M0
#    GPIO 27 → M1

# 2. 점퍼로 수동 모드 설정
#    Normal: M0=GND, M1=GND
#    Config: M0=GND, M1=3.3V
```

---

### ❌ 설정 읽기/쓰기 실패

**증상:**
```
❌ 읽기 실패 또는 응답 오류
FF FF FF
```

**해결:**
```bash
# 1. Config 모드 확인
sudo python3 e22_mode.py config

# 2. 잠시 대기
sleep 2

# 3. 재시도
sudo python3 e22_config.py --verify

# 4. 하드 리셋 (RST 핀 사용)
# 또는 전원 재연결
```

---

### ❌ 통신 안됨 (송수신 실패)

**체크리스트:**

1. **모드 확인**
   ```bash
   # Normal 모드여야 송수신 가능
   sudo python3 e22_mode.py normal
   ```

2. **채널 일치**
   ```bash
   # 양쪽 기기 모두 동일한 채널
   sudo python3 e22_config.py --channel 0x32 --verify
   ```

3. **네트워크 ID 일치**
   ```bash
   # 양쪽 기기 모두 동일한 NetID
   sudo python3 e22_config.py --netid 0x00 --verify
   ```

4. **무선 속도 일치**
   ```bash
   # 양쪽 기기 모두 동일한 Air Data Rate
   sudo python3 e22_config.py --adr 2.4k --verify
   ```

5. **안테나 연결 확인**
   - 안테나가 제대로 연결되어 있는지 확인

---

### 📊 RSSI 값으로 신호 품질 확인

```bash
# RSSI 로깅으로 통신 상태 확인
sudo python3 e22_config.py --rssi-log
```

**RSSI 값 해석:**
- `-30 ~ -60 dBm`: 매우 좋음 ✅
- `-60 ~ -80 dBm`: 좋음 ✅
- `-80 ~ -100 dBm`: 보통 ⚠️
- `-100 dBm 이하`: 약함 ❌

---

## 📌 주요 명령어 요약

```bash
# === 모드 변경 ===
sudo python3 e22_mode.py normal    # 송수신 모드
sudo python3 e22_mode.py config    # 설정 모드

# === 설정 확인 ===
sudo python3 e22_config.py --verify

# === 기본 설정 ===
sudo python3 e22_config.py \
  --addr 0x0000 --netid 0x00 --channel 0x32 \
  --baud 9600 --adr 2.4k --mode save --verify

# === RSSI 측정 ===
sudo python3 e22_config.py --rssi-log --interval 1.0

# === 투명 모드 ===
sudo python3 e22_trans.py --save --verify_after

# === 송수신 테스트 ===
cd SX126X_LoRa_HAT_Code/raspberrypi/python
sudo python3 main.py
```

---

## 🔧 고급 설정

### 시작 시 자동 실행

```bash
# 1. systemd 서비스 파일 생성
sudo nano /etc/systemd/system/lora.service

# 2. 내용 입력:
[Unit]
Description=LoRa Service
After=network.target

[Service]
Type=simple
User=pi
WorkingDirectory=/home/pi/jm/SX126X_LoRa_HAT_Code/raspberrypi/python
ExecStart=/usr/bin/python3 main.py
Restart=always

[Install]
WantedBy=multi-user.target

# 3. 서비스 활성화
sudo systemctl enable lora.service
sudo systemctl start lora.service
sudo systemctl status lora.service
```

---

## 📚 추가 자료

- **공식 GitHub:** https://github.com/MithunHub/LoRa
- **데이터시트:** E22-xxxT22S LoRa 모듈 데이터시트
- **라즈베리파이 GPIO:** https://pinout.xyz

---

## 📝 참고사항

### 통신 범위
- **실내:** 약 100~500m
- **실외 (장애물 없음):** 최대 3~5km
- **출력:** 22dBm (약 160mW)

### 주파수 대역
- **E22-400T22S:** 410~493 MHz
- **E22-900T22S:** 850~930 MHz (주로 사용)

### 전력 소비
- **송신:** 약 120mA @ 22dBm
- **수신:** 약 15mA
- **절전 (Sleep):** 약 2μA

---

## ✅ 체크리스트 (새 기기 설정 시)

- [ ] 하드웨어 연결 (UART, GND, VCC)
- [ ] M0, M1 연결 (또는 점퍼 설정)
- [ ] 시리얼 포트 활성화 (`raspi-config`)
- [ ] Python 패키지 설치
- [ ] 사용자 권한 설정 (`dialout` 그룹)
- [ ] Config 모드로 전환
- [ ] 주소 및 채널 설정
- [ ] Normal 모드로 전환
- [ ] 송수신 테스트

---

**작성일:** 2025-11-03  
**버전:** 1.0  
**작성자:** AI Assistant

문제가 발생하면 이 문서의 [문제 해결](#5-문제-해결) 섹션을 참고하세요! 🚀

