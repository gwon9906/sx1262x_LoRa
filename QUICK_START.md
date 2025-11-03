# ⚡ LoRa HAT 빠른 시작 가이드

## 🔌 1. 하드웨어 연결

```
라즈베리파이          LoRa HAT
────────────────────────────
Pin 8  (TX)    →     RXD
Pin 10 (RX)    →     TXD
Pin 11 (GPIO17)→     M0  (선택)
Pin 13 (GPIO27)→     M1  (선택)
Pin 6  (GND)   →     GND
Pin 2  (5V)    →     VCC
```

> **M0/M1 연결 없이 사용:** 점퍼로 M0=GND, M1=GND 고정

---

## ⚙️ 2. 시스템 설정

```bash
# 시리얼 포트 활성화
sudo raspi-config
# Interface Options → Serial Port
# Login shell: NO, Serial: YES
# 재부팅

# 권한 설정
sudo usermod -a -G dialout $USER
# 로그아웃 후 재로그인
```

---

## 🚀 3. 기본 사용법

### 설정 확인
```bash
sudo python3 e22_mode.py config
sudo python3 e22_config.py --verify
```

### 설정 변경
```bash
sudo python3 e22_config.py \
  --addr 0x0000 \
  --channel 0x32 \
  --mode save \
  --verify
```

### 송수신 모드
```bash
sudo python3 e22_mode.py normal
cd SX126X_LoRa_HAT_Code/raspberrypi/python
sudo python3 main.py
```

---

## 💡 핵심 명령어

| 명령 | 설명 |
|------|------|
| `sudo python3 e22_mode.py normal` | 송수신 모드 |
| `sudo python3 e22_mode.py config` | 설정 모드 |
| `sudo python3 e22_config.py --verify` | 현재 설정 확인 |
| `sudo python3 e22_config.py --rssi-log` | RSSI 측정 |

---

## ⚠️ 주의사항

✅ **송수신 전:** 양쪽 기기의 **채널, 네트워크ID, 무선속도** 동일하게 설정  
✅ **모드 변경 후:** 2~3초 대기  
✅ **안테나:** 반드시 연결 (안테나 없이 송신 시 모듈 손상 가능)

자세한 내용은 `README_LORA.md` 참고!

