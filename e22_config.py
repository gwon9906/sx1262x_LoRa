import serial
import argparse
import time
import sys
import logging
import csv
from datetime import datetime

# ---------- 설정 가능한 값 매핑 ----------

BAUD_BITS = {
    1200: 0b000,
    2400: 0b001,
    4800: 0b010,
    9600: 0b011,
    19200: 0b100,
    38400: 0b101,
    57600: 0b110,
    115200: 0b111,
}

PARITY_BITS = {
    "8N1": 0b00,
    "8O1": 0b01,
    "8E1": 0b10,
}

ADR_BITS = {
    "0.3k": 0b000,
    "1.2k": 0b001,
    "2.4k": 0b010,
    "4.8k": 0b011,
    "9.6k": 0b100,
    "19.2k": 0b101,
    "38.4k": 0b110,
    "62.5k": 0b111,
}

PACKET_SIZE_BITS = {
    240: 0b00,
    128: 0b01,
    64: 0b10,
    32: 0b11,
}

POWER_BITS = {
    22: 0b00,  # 22dBm
    17: 0b01,  # 17dBm
    13: 0b10,  # 13dBm
    10: 0b11,  # 10dBm
}

BAUD_REV = {v: k for k, v in BAUD_BITS.items()}
PARITY_REV = {v: k for k, v in PARITY_BITS.items()}
ADR_REV = {v: k for k, v in ADR_BITS.items()}
PACKET_SIZE_REV = {v: k for k, v in PACKET_SIZE_BITS.items()}
POWER_REV = {v: k for k, v in POWER_BITS.items()}


def build_reg0(baud, parity, adr):
    return (BAUD_BITS[baud] << 5) | (PARITY_BITS[parity] << 3) | ADR_BITS[adr]


def build_reg1(packet_size=240, rssi_noise=False, power=22):
    """REG1 빌드: 패킷 크기, RSSI 노이즈, 송신 출력"""
    packet_bits = PACKET_SIZE_BITS[packet_size]
    rssi_bit = 1 if rssi_noise else 0
    power_bits = POWER_BITS[power]
    # Bit 7-6: Packet Size, Bit 5: RSSI Noise, Bit 4-3: Reserved(0), Bit 1-0: Power
    return (packet_bits << 6) | (rssi_bit << 5) | power_bits


def build_reg3(transparent_mode=True, rssi_output=False, relay=False, lbt=False, wor_period=0b000):
    """REG3 빌드: 전송 모드, RSSI 출력, 릴레이, LBT, WOR"""
    rssi_out_bit = 1 if rssi_output else 0
    transfer_bit = 0 if transparent_mode else 1  # 0: 투명 모드, 1: 고정점 모드
    relay_bit = 1 if relay else 0
    lbt_bit = 1 if lbt else 0
    wor_ctrl = 0  # WOR 송신기 (0) or 수신기 (1)
    # Bit 7: RSSI, Bit 6: Transfer, Bit 5: Relay, Bit 4: LBT, Bit 3: WOR Ctrl, Bit 2-0: WOR Period
    return (rssi_out_bit << 7) | (transfer_bit << 6) | (relay_bit << 5) | (lbt_bit << 4) | (wor_ctrl << 3) | (wor_period & 0b111)


def send_config(cmd_type, addr_high, addr_low, netid, reg0, reg1, reg2, reg3, port, baudrate):
    base_cmd = []
    if cmd_type == "save":
        base_cmd = [0xC0]
    elif cmd_type == "temp":
        base_cmd = [0xC2]
    elif cmd_type == "wireless":
        base_cmd = [0xCF, 0xCF, 0xC2]
    else:
        raise ValueError("명령 형식 오류: save/temp/wireless 중 하나여야 함")

    # 전체 7바이트 설정: ADDH, ADDL, NETID, REG0, REG1, REG2, REG3
    packet = bytes(base_cmd + [0x00, 0x07, addr_high, addr_low, netid, reg0, reg1, reg2, reg3])
    print(f"▶️ 전송 ({cmd_type}): {packet.hex().upper()}")

    with serial.Serial(port, baudrate, timeout=1) as ser:
        ser.write(packet)
        time.sleep(0.2)
        resp = ser.read_all()
        print(f"✅ 응답: {resp.hex().upper()}")

        if resp.startswith(b'\xFF\xFF\xFF'):
            print("❌ 포맷 오류: FF FF FF")
        elif resp.startswith(b'\xC1'):
            print("✅ 설정 성공")
        elif resp.startswith(b'\xCF\xCF\xC1'):
            print("✅ 무선 설정 성공")
        else:
            print("⚠️ 알 수 없는 응답")


def read_config(port, baudrate):
    read_cmd = bytes([0xC1, 0x00, 0x07])  # 7바이트 읽기
    with serial.Serial(port, baudrate, timeout=1) as ser:
        ser.write(read_cmd)
        time.sleep(0.2)
        resp = ser.read_all()
        print(f"\n📥 읽기 응답: {resp.hex().upper()}")

        if not resp.startswith(b'\xC1\x00\x07') or len(resp) < 10:
            print("❌ 읽기 실패 또는 응답 오류")
            return

        addr_high = resp[3]
        addr_low = resp[4]
        netid = resp[5]
        reg0 = resp[6]
        reg1 = resp[7]
        reg2 = resp[8]
        reg3 = resp[9]

        # REG0 해석
        baud = BAUD_REV[(reg0 >> 5) & 0b111]
        parity = PARITY_REV[(reg0 >> 3) & 0b11]
        adr = ADR_REV[reg0 & 0b111]

        # REG1 해석
        packet_size = PACKET_SIZE_REV.get((reg1 >> 6) & 0b11, "Unknown")
        rssi_noise = bool((reg1 >> 5) & 0b1)
        power = POWER_REV.get(reg1 & 0b11, "Unknown")

        # REG2 (채널)
        freq = 850.125 + reg2 * 1.0

        # REG3 해석
        rssi_output = bool((reg3 >> 7) & 0b1)
        transfer_mode = "투명(Transparent)" if not bool((reg3 >> 6) & 0b1) else "고정점(Fixed)"
        relay = bool((reg3 >> 5) & 0b1)
        lbt = bool((reg3 >> 4) & 0b1)
        wor_period = reg3 & 0b111

        print("📊 현재 설정:")
        print(f"  주소     : 0x{addr_high:02X}{addr_low:02X}")
        print(f"  네트워크 : 0x{netid:02X}")
        print(f"  UART     : {baud} bps")
        print(f"  패리티   : {parity}")
        print(f"  무선속도 : {adr}")
        print(f"  채널     : 0x{reg2:02X} ({reg2} → {freq:.3f} MHz)")
        print(f"  패킷크기 : {packet_size} bytes")
        print(f"  송신출력 : {power} dBm")
        print(f"  전송모드 : {transfer_mode}")
        print(f"  RSSI출력 : {'활성화' if rssi_output else '비활성화'}")
        print(f"  릴레이   : {'활성화' if relay else '비활성화'}")
        print(f"  LBT      : {'활성화' if lbt else '비활성화'}")


# ---------- RSSI 로깅 전용 ----------

READ_RSSI_CMD = bytes([0xC0, 0xC1, 0xC2, 0xC3, 0x00, 0x01])

def _read_noise_rssi(ser):
    """Ambient RSSI(dBm) 한 번 읽기 (실패 시 None)."""
    ser.write(READ_RSSI_CMD)
    time.sleep(0.05)
    resp = ser.read(4)          # 기대: C1 00 01 RSSI
    if resp[:3] == b'\xC1\x00\x01' and len(resp) == 4:
        raw = resp[3]
        return -(256 - raw)
    return None

def rssi_log_loop(port, baudrate, interval, csv_path):
    """
    주기적으로 RSSI를 읽어 터미널과(선택) CSV에 기록한다.
    ⌃C 로 중단.
    """
    logging.info(
        "RSSI logging 시작 → port=%s baud=%s interval=%.2fs csv=%s",
        port, baudrate, interval, csv_path or "없음")

    ser = serial.Serial(port, baudrate, timeout=0.2)

    csv_file = open(csv_path, "a", newline="") if csv_path else None
    writer = csv.writer(csv_file) if csv_file else None
    if writer and csv_file.tell() == 0:
        writer.writerow(["timestamp", "rssi_dbm"])

    try:
        while True:
            rssi = _read_noise_rssi(ser)
            if rssi is not None:
                ts = datetime.now().isoformat(timespec="seconds")
                msg = f"[{ts}] RSSI = {rssi} dBm"
                print(msg)
                logging.info(msg)
                if writer:
                    writer.writerow([ts, rssi])
            time.sleep(interval)
    except KeyboardInterrupt:
        print("\n⏹️  RSSI logging 종료")
    finally:
        ser.close()
        if csv_file:
            csv_file.close()


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="E22 설정 전송 + 채널 포함")
    parser.add_argument("--addr", type=lambda x: int(x, 16), default=0x0000,
                    help="모듈 주소(16진, 예: 0x0001)")
    parser.add_argument("--netid", type=lambda x: int(x, 16), default=0x00)
    parser.add_argument("--baud", type=int, choices=BAUD_BITS.keys(), default=9600)
    parser.add_argument("--parity", type=str, choices=PARITY_BITS.keys(), default="8N1")
    parser.add_argument("--adr", type=str, choices=ADR_BITS.keys(), default="2.4k")
    parser.add_argument("--channel", type=lambda x: int(x, 16), default=0x32, help="채널 (0x00~0x50)")
    parser.add_argument("--packet-size", type=int, choices=PACKET_SIZE_BITS.keys(), default=240, help="패킷 크기")
    parser.add_argument("--power", type=int, choices=POWER_BITS.keys(), default=22, help="송신 출력 (dBm)")
    parser.add_argument("--transparent", action="store_true", help="투명 전송 모드 (기본값)")
    parser.add_argument("--fixed", action="store_true", help="고정점 전송 모드")
    parser.add_argument("--port", type=str, default="/dev/ttyAMA0")
    parser.add_argument("--mode", type=str, choices=["save", "temp", "wireless"], default="save")
    parser.add_argument("--verify", action="store_true")
    parser.add_argument("--rssi-out", action="store_true",
                    help="REG3 Bit7: RSSI 바이트 출력 활성화")
    parser.add_argument("--rssi-log", action="store_true",
                        help="RSSI 로깅 모드 (다른 설정 단계 건너뜀)")
    parser.add_argument("--interval", type=float, default=1.0,
                        help="RSSI 측정 간격(초)")
    parser.add_argument("--csv", type=str, metavar="PATH",
                        help="RSSI 값을 저장할 CSV 경로")
    parser.add_argument("--loglevel", default="INFO",
                        choices=["DEBUG", "INFO", "WARNING", "ERROR", "CRITICAL"],
                        help="로그 레벨")

    args = parser.parse_args()
    
    logging.basicConfig(
        level=getattr(logging, args.loglevel),
        format="%(asctime)s [%(levelname)s] %(message)s"
    )

    if args.rssi_log:
        rssi_log_loop(args.port, args.baud, args.interval, args.csv)
        sys.exit(0)

    # 레지스터 빌드
    reg0 = build_reg0(args.baud, args.parity, args.adr)
    reg1 = build_reg1(packet_size=args.packet_size, rssi_noise=False, power=args.power)
    reg2 = args.channel & 0xFF
    
    # 전송 모드: --fixed 옵션이 있으면 고정점 모드, 없으면 투명 모드
    transparent_mode = not args.fixed
    
    # RSSI 출력 여부
    rssi_output = args.rssi_out   # ← 이 줄을 추가 또는 수정
    
    # REG3 생성
    reg3 = build_reg3(
        transparent_mode=transparent_mode,
        rssi_output=rssi_output,
        relay=False,
        lbt=False
    )
    
    addr_high = (args.addr >> 8) & 0xFF
    addr_low = args.addr & 0xFF

    print(f"\n📦 설정 요약:")
    print(f"  REG0 = 0x{reg0:02X} (UART: {args.baud} bps, {args.parity}, 무선속도: {args.adr})")
    print(f"  REG1 = 0x{reg1:02X} (패킷: {args.packet_size}B, 출력: {args.power}dBm)")
    print(f"  REG2 = 0x{reg2:02X} (채널: {reg2})")
    print(f"  REG3 = 0x{reg3:02X} (전송모드: {'투명' if transparent_mode else '고정점'})")
    print(f"  주소 = 0x{args.addr:04X}, 네트워크 = 0x{args.netid:02X}")
    
    send_config(args.mode, addr_high, addr_low, args.netid, reg0, reg1, reg2, reg3, args.port, args.baud)

    if args.verify:
        read_config(args.port, args.baud)
    
