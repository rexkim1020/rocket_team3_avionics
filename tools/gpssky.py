# GPS 수신 품질 진단 (실외용). SD·서보 건드리지 않음.
#
#   "문장은 오는데 픽스가 안 잡힌다" 의 원인을 가른다.
#     - 위성 수는 많은데 SNR 이 낮다  -> 안테나가 가려짐/불량/방향 문제
#     - 위성 수 자체가 적다           -> 하늘 시야 부족(건물·나무·동체)
#     - 시간이 지나며 좋아진다        -> 콜드스타트, 더 기다리면 됨
#
#   GSV 문장에서 위성별 SNR(dBHz)을 읽는다. 보통:
#     40 이상 = 아주 좋음 / 30~40 = 양호 / 25 이하 = 픽스 어려움
#
#   사용법 (실외에서, Thonny Shell):
#       import gpssky
#       gpssky.run(300)      # 5분
from machine import UART, Pin
from time import sleep_ms, ticks_ms, ticks_diff
import config


def _cks_ok(s):
    if not s.startswith("$") or "*" not in s:
        return False
    body, _x, c = s[1:].partition("*")
    v = 0
    for ch in body:
        v ^= ord(ch)
    try:
        return v == int(c[:2], 16)
    except Exception:
        return False


def run(seconds=300):
    u = UART(config.GPS_UART_ID, baudrate=config.GPS_BAUD,
             tx=Pin(config.GPS_TX_PIN), rx=Pin(config.GPS_RX_PIN))
    sleep_ms(300)
    print("GPS 수신 품질 진단 %d초. 하늘이 최대한 넓게 보이는 곳에 두세요." % seconds)
    print("%6s %5s %6s %7s %8s  %s" % ("t(s)", "fix", "사용중", "보임", "SNR상위", "비고"))

    buf = b""
    t0 = ticks_ms()
    last = t0
    fix = 0
    used = 0
    inview = 0
    snrs = {}
    best_fix_t = None
    max_snr = 0
    while ticks_diff(ticks_ms(), t0) < seconds * 1000:
        n = u.any()
        if n:
            d = u.read(n)
            if d:
                buf += d
                while b"\n" in buf:
                    line, buf = buf.split(b"\n", 1)
                    try:
                        s = line.decode().strip()
                    except Exception:
                        continue
                    if not _cks_ok(s):
                        continue
                    f = s[1:].split("*")[0].split(",")
                    t = f[0][-3:]
                    if t == "GGA":
                        try:
                            fix = int(f[6]) if f[6] else 0
                            used = int(f[7]) if f[7] else 0
                        except Exception:
                            pass
                    elif t == "GSV":
                        try:
                            inview = int(f[3]) if f[3] else 0
                            i = 4
                            while i + 3 < len(f):
                                prn = f[i]
                                sn = f[i + 3]
                                if prn and sn:
                                    snrs[prn] = int(sn)
                                i += 4
                        except Exception:
                            pass
        if len(buf) > 1024:
            buf = buf[-512:]

        now = ticks_ms()
        if ticks_diff(now, last) >= 5000:
            last = now
            el = ticks_diff(now, t0) // 1000
            top = sorted(snrs.values(), reverse=True)[:4]
            if top and top[0] > max_snr:
                max_snr = top[0]
            note = ""
            if fix > 0 and best_fix_t is None:
                best_fix_t = el
                note = "★픽스 획득!"
            print("%6d %5d %6d %7d %8s  %s"
                  % (el, fix, used, inview,
                     "/".join(str(v) for v in top) if top else "-", note))
            snrs = {}

    print("")
    print("=== 결과 ===")
    print("  최고 SNR %d dBHz / 보이는 위성 %d개 / 픽스 %s"
          % (max_snr, inview,
             ("%d초에 획득" % best_fix_t) if best_fix_t is not None else "실패"))
    print("")
    if max_snr == 0:
        print("  -> GSV 문장이 없거나 위성이 전혀 안 보임. 안테나 연결 확인")
    elif max_snr < 25:
        print("  -> ★신호가 매우 약함. 안테나가 가려졌거나(동체/노즈콘) 불량,")
        print("     또는 안테나 면이 하늘을 향하지 않음")
    elif max_snr < 32:
        print("  -> 신호가 약한 편. 하늘 시야를 넓히면 개선 가능")
    else:
        print("  -> 신호 세기는 충분. 픽스가 안 되면 더 기다려보세요(콜드스타트)")
