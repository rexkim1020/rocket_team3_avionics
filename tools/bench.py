# ============================================================
#  벤치 테스트 스크립트 (REPL 대화형)
#  발사 전 실측 체크리스트를 여기서 전부 처리한다.
#
#  사용법:  >>> import bench
#           >>> bench.help()
#
#  ★ 주의: 보드에 비행용 main.py 가 올라가 있으면 부팅 시 자동 실행되고
#    워치독(WDT)이 걸려 REPL 작업 중 8초마다 리셋된다.
#    벤치 작업 중에는 main.py 를 보드에서 빼두거나 config.WDT_ENABLED=False 로.
#    (비행 테스트는  mpremote run main.py  로 직접 실행)
# ============================================================
from machine import Pin, I2C, PWM, UART, SPI
from time import sleep_ms, ticks_ms, ticks_diff
import math
import config

_bmp = None
_bno = None
_servo = None


def help():
    print("""
=== GoRocket 벤치 테스트 ===
 [진단]
  bench.check()        전체 요약 (센서/SD/GPS 한 번에)
  bench.scan()         I2C 버스 2개 장치 스캔
  bench.sd()           SD 마운트 + 쓰기/읽기 테스트
 [★축 설정 - 가장 중요]
  bench.axis()         로켓 수직(노즈 위)으로 세우고 실행 -> config 값 알려줌
 [센서 확인]
  bench.imu(n)         가속도/각속도 실시간 (기본 30회)
  bench.tilt(s)        tilt 실시간 (자이로 적분, 기본 20초)
  bench.baro(n)        기압/고도 실시간 (기본 30회)
  bench.vel(s)         속도 비교: 기압미분 vs IMU적분 (기본 20초)
 [GPS]
  bench.gps(s)         픽스/위성수/좌표 실시간 (기본 60초)
  bench.gpsbaud()      보율 자동 탐색 (GPS_BAUD 확인용)
 [구동부]
  bench.servo(us)      서보 펄스 직접 지정 (예: bench.servo(1500))
  bench.angle(deg)     각도로 지정 (예: bench.angle(90))
  bench.safe()         config SERVO_SAFE_US 로
  bench.deploy()       config SERVO_DEPLOY_US 로
  bench.sweep(a,b)     a~b us 천천히 왕복
  bench.servo_off()    PWM 정지(서보 힘 빼기)
  bench.buzzer(ms)     부저 테스트
 [입력]
  bench.umbilical(n)   엄빌리컬 3개 상태 + 판정 스테이지 실시간
""")


# ---------- 지연 초기화 ----------
def bmp():
    global _bmp
    if _bmp is None:
        from bmp388 import BMP388
        i2c = I2C(config.BMP_I2C_ID, sda=Pin(config.BMP_SDA_PIN),
                  scl=Pin(config.BMP_SCL_PIN), freq=config.I2C_FREQ)
        _bmp = BMP388(i2c)
        print("[OK] BMP388 @", hex(_bmp.address))
    return _bmp


def bno():
    global _bno
    if _bno is None:
        from bno055 import BNO055
        i2c = I2C(config.BNO_I2C_ID, sda=Pin(config.BNO_SDA_PIN),
                  scl=Pin(config.BNO_SCL_PIN), freq=config.I2C_FREQ)
        _bno = BNO055(i2c, config.IMU_AXIAL_AXIS)
        print("[OK] BNO055 @", hex(_bno.address), "(AMG +-16g)")
    return _bno


# ---------- 진단 ----------
def scan():
    for name, ident, sda, scl in (
        ("I2C%d (BMP388)" % config.BMP_I2C_ID, config.BMP_I2C_ID,
         config.BMP_SDA_PIN, config.BMP_SCL_PIN),
        ("I2C%d (BNO055)" % config.BNO_I2C_ID, config.BNO_I2C_ID,
         config.BNO_SDA_PIN, config.BNO_SCL_PIN),
    ):
        try:
            i2c = I2C(ident, sda=Pin(sda), scl=Pin(scl), freq=config.I2C_FREQ)
            found = [hex(a) for a in i2c.scan()]
            print("%-16s GP%d/GP%d -> %s" % (name, sda, scl, found if found else "없음"))
        except Exception as e:
            print("%-16s 실패: %s" % (name, e))
    print("기대값: BMP388 = 0x76 또는 0x77 / BNO055 = 0x28 또는 0x29")


def sd():
    import os
    try:
        import sdcard
    except Exception as e:
        print("[SD] sdcard.py 없음:", e)
        return
    try:
        spi = SPI(config.SD_SPI_ID, baudrate=1_000_000,
                  sck=Pin(config.SD_SCK_PIN), mosi=Pin(config.SD_MOSI_PIN),
                  miso=Pin(config.SD_MISO_PIN))
        sleep_ms(250)
        card = sdcard.SDCard(spi, Pin(config.SD_CS_PIN))
        try:
            os.mount(card, config.SD_MOUNT)
            print("[SD] 마운트 OK")
        except Exception as e:
            print("[SD] 마운트 건너뜀(이미 마운트됐을 수 있음):", e)
        print("[SD] 파일 목록:", os.listdir(config.SD_MOUNT))
        path = config.SD_MOUNT + "/bench_test.txt"
        f = open(path, "w")
        f.write("gorocket")
        f.close()
        f = open(path, "r")
        back = f.read()
        f.close()
        os.remove(path)
        print("[SD] 쓰기/읽기", "OK" if back == "gorocket" else "불일치! (%s)" % back)
    except Exception as e:
        print("[SD] 실패:", e)
        print("     확인: FAT32 포맷인지 / 배선(GP10 SCK,11 MOSI,12 MISO,13 CS) / 카드 접점")


def check():
    print("--- 전체 요약 ---")
    scan()
    try:
        p, t = bmp().read()
        print("BMP388  : %.1f Pa, %.2f C" % (p, t))
    except Exception as e:
        print("BMP388  : 실패 -", e)
    try:
        a, w = bno().read_accel_gyro()
        print("BNO055  : accel %.2f %.2f %.2f m/s^2 | gyro %.2f %.2f %.2f dps"
              % (a[0], a[1], a[2], w[0], w[1], w[2]))
        print("          |accel| = %.2f (정지 시 9.8 근처여야 정상)"
              % math.sqrt(a[0]**2 + a[1]**2 + a[2]**2))
    except Exception as e:
        print("BNO055  : 실패 -", e)
    sd()
    gps(6)


# ---------- ★ 축/부호 판별 ----------
def axis(samples=100):
    print("로켓을 발사 자세(노즈 위, 수직)로 고정하고 흔들지 마세요. 측정 중...")
    s = [0.0, 0.0, 0.0]
    n = 0
    for _ in range(samples):
        try:
            a, _w = bno().read_accel_gyro()
            for i in range(3):
                s[i] += a[i]
            n += 1
        except Exception:
            pass
        sleep_ms(20)
    if n == 0:
        print("IMU 읽기 실패")
        return
    avg = [v / n for v in s]
    print("평균 가속도: X=%+.3f  Y=%+.3f  Z=%+.3f  (m/s^2)" % (avg[0], avg[1], avg[2]))

    idx = 0
    for i in (1, 2):
        if abs(avg[i]) > abs(avg[idx]):
            idx = i
    mag = abs(avg[idx])
    sign = 1.0 if avg[idx] > 0 else -1.0
    names = ("X", "Y", "Z")

    # 축방향 성분이 충분히 지배적인지(=제대로 수직인지) 확인
    other = math.sqrt(sum(avg[i] ** 2 for i in range(3) if i != idx))
    tilt = math.degrees(math.atan2(other, mag))

    print("")
    print("축방향 = %s축 (인덱스 %d), 크기 %.2f m/s^2, 기울기 약 %.1f도" % (names[idx], idx, mag, tilt))
    if mag < 8.5 or mag > 11.0:
        print("!! 크기가 9.8에서 많이 벗어남 - 센서 설정/배선 확인 필요")
    if tilt > 10.0:
        print("!! 수직이 아님 - 더 똑바로 세우고 다시 실행하세요")
    print("")
    print("config.py 에 아래처럼 넣으세요:")
    print("    IMU_AXIAL_AXIS = %d      # %s축" % (idx, names[idx]))
    print("    IMU_AXIAL_SIGN = %.1f    # 노즈 위에서 +가 되도록" % sign)
    print("")
    print("검산: 이 설정이면 ground_accel = %+.2f (양수 9.8 근처여야 정답)"
          % (avg[idx] * sign))


# ---------- 센서 실시간 ----------
def imu(n=30):
    for _ in range(n):
        try:
            a, w = bno().read_accel_gyro()
            g = math.sqrt(a[0]**2 + a[1]**2 + a[2]**2)
            ax_v = a[config.IMU_AXIAL_AXIS] * config.IMU_AXIAL_SIGN
            print("a=%+7.2f %+7.2f %+7.2f |%5.2f|  w=%+7.1f %+7.1f %+7.1f  축방향=%+6.2f"
                  % (a[0], a[1], a[2], g, w[0], w[1], w[2], ax_v))
        except Exception as e:
            print("실패:", e)
        sleep_ms(200)


def tilt(seconds=20):
    from attitude import Attitude
    print("자이로 바이어스 측정 중(움직이지 마세요, 2초)...")
    bias = [0.0, 0.0, 0.0]
    n = 0
    for _ in range(100):
        try:
            _a, w = bno().read_accel_gyro()
            for i in range(3):
                bias[i] += w[i]
            n += 1
        except Exception:
            pass
        sleep_ms(20)
    if n:
        bias = [v / n for v in bias]
    print("bias = %+.3f %+.3f %+.3f dps" % (bias[0], bias[1], bias[2]))
    print("지금 자세가 0도 기준입니다. 90도 눕히면 90이 나와야 정상. (%d초)" % seconds)

    # ★ 적분은 비행과 같은 20ms 주기로 돌린다.
    #   Attitude.update()는 dt>0.2s 를 버리므로 200ms 주기로 돌리면
    #   모든 갱신이 무시되어 tilt가 0에서 안 움직인다.
    att = Attitude(config.IMU_AXIAL_AXIS)
    last = ticks_ms()
    t_end = ticks_ms() + seconds * 1000
    i = 0
    while ticks_diff(t_end, ticks_ms()) > 0:
        now = ticks_ms()
        dt = ticks_diff(now, last) / 1000.0
        last = now
        try:
            a, w = bno().read_accel_gyro()
            att.update(w[0] - bias[0], w[1] - bias[1], w[2] - bias[2], dt)
            i += 1
            if i % 10 == 0:                 # 출력만 200ms 간격
                print("자이로적분 tilt=%6.2f도   (정지기준 가속도 tilt=%s)"
                      % (att.tilt_deg(), _fmt(bno().tilt_from_accel(a))))
        except Exception as e:
            print("실패:", e)
        sleep_ms(20)


def _fmt(v):
    return "----" if v is None else "%.2f" % v


def vel(seconds=20):
    # 두 속도(기압 미분 vs IMU 적분)를 나란히 확인.
    # 보드를 위아래로 빠르게 들었다 놨다 하면 두 값이 같이 움직여야 정상.
    from velocity import BaroVelocity
    from attitude import Attitude
    import math as _m
    print("기준값 측정(2초, 움직이지 마세요)...")
    aref = [0.0, 0.0, 0.0]
    bias = [0.0, 0.0, 0.0]
    n = 0
    for _ in range(100):
        try:
            a, w = bno().read_accel_gyro()
            for k in range(3):
                aref[k] += a[k]
                bias[k] += w[k]
            n += 1
        except Exception:
            pass
        sleep_ms(20)
    if n:
        aref = [v / n for v in aref]
        bias = [v / n for v in bias]
    # 중력 방향 단위벡터(3축 전부 사용 -> X/Y 바이어스·기울기까지 흡수)
    g0 = _m.sqrt(aref[0] ** 2 + aref[1] ** 2 + aref[2] ** 2)
    u = [v / g0 for v in aref] if g0 > 1.0 else [0.0, 0.0, 1.0]
    ref = bmp().read()[0]
    print("중력벡터 = %+.3f %+.3f %+.3f  |g|=%.3f m/s^2" % (aref[0], aref[1], aref[2], g0))
    print("기준압력 = %.1f Pa" % ref)
    print("지금 자세가 '연직' 기준입니다. 위아래로 움직여보세요. (%d초)" % seconds)

    bv = BaroVelocity()
    att = Attitude(config.IMU_AXIAL_AXIS)
    v_imu = 0.0
    last = ticks_ms()
    t_end = ticks_ms() + seconds * 1000
    i = 0
    while ticks_diff(t_end, ticks_ms()) > 0:
        now = ticks_ms()
        dt = ticks_diff(now, last) / 1000.0
        last = now
        try:
            p, _t = bmp().read()
            alt = 44330.0 * (1.0 - _m.pow(p / ref, 0.19029495718))
            vb = bv.update(now, alt)
            a, w = bno().read_accel_gyro()
            att.update(w[0] - bias[0], w[1] - bias[1], w[2] - bias[2], dt)
            if config.IMU_VERTICAL_PROJECTION:
                r = att.rotate(a[0], a[1], a[2])
            else:
                r = a
            acc = r[0] * u[0] + r[1] * u[1] + r[2] * u[2] - g0   # 중력방향 성분
            if 0.0 < dt < 0.2:
                v_imu += acc * dt
            i += 1
            if i % 10 == 0:
                print("고도%+7.2fm  기울기%5.1f도  IMU v=%+7.2f  기압 v=%s  차이=%s"
                      % (alt, att.tilt_deg(), v_imu, _fmt(vb),
                         _fmt(None if vb is None else v_imu - vb)))
        except Exception as e:
            print("실패:", e)
        sleep_ms(20)
    print("\n※ 연직투영 %s. 보드를 기울여도 IMU 속도가 크게 안 튀어야 정상."
          % ("ON" if config.IMU_VERTICAL_PROJECTION else "OFF"))
    print("  기압 미분은 정지 시 0 근처를 유지해야 한다.")


def baro(n=30):
    try:
        ref = bmp().read()[0]
    except Exception as e:
        print("BMP388 실패:", e)
        return
    print("기준압력 %.1f Pa 로 영점. 보드를 위아래로 움직여보세요." % ref)
    for _ in range(n):
        try:
            p, t = bmp().read()
            alt = 44330.0 * (1.0 - math.pow(p / ref, 0.19029495718))
            print("P=%9.1f Pa  T=%5.2f C  상대고도=%+7.2f m" % (p, t, alt))
        except Exception as e:
            print("실패:", e)
        sleep_ms(300)


# ---------- GPS ----------
def _nmea_ok(line):
    try:
        s = line.decode().strip()
    except Exception:
        return False
    if not s.startswith("$") or "*" not in s:
        return False
    body, _sep, cks = s[1:].partition("*")
    c = 0
    for ch in body:
        c ^= ord(ch)
    try:
        return c == int(cks[:2], 16)
    except Exception:
        return False


def gpsbaud(bauds=(9600, 4800, 38400, 57600, 115200), seconds=3):
    print("각 보율로 %d초씩 들어봅니다..." % seconds)
    best = None
    for b in bauds:
        u = UART(config.GPS_UART_ID, baudrate=b,
                 tx=Pin(config.GPS_TX_PIN), rx=Pin(config.GPS_RX_PIN))
        sleep_ms(200)
        buf = b""
        good = 0
        t_end = ticks_ms() + seconds * 1000
        while ticks_diff(t_end, ticks_ms()) > 0:
            if u.any():
                d = u.read(u.any())
                if d:
                    buf += d
                    while b"\n" in buf:
                        line, buf = buf.split(b"\n", 1)
                        if _nmea_ok(line):
                            good += 1
            sleep_ms(20)
        print("  %6d baud -> 유효 NMEA %d 문장 %s" % (b, good, "  <== 이것!" if good else ""))
        if good and (best is None or good > best[1]):
            best = (b, good)
    if best:
        print("\nconfig.py 에:  GPS_BAUD = %d" % best[0])
    else:
        print("\n어느 보율에서도 수신 없음. 확인: 5V 전원 / TX-RX 교차(GP4->GPS RX, GP5<-GPS TX)")


def gps(seconds=60):
    from gps import GPS
    g = GPS()
    print("GPS 수신 중... (실외/하늘 보이는 곳에서 콜드스타트 30초~수분)")
    t_end = ticks_ms() + seconds * 1000
    while ticks_diff(t_end, ticks_ms()) > 0:
        g.update()
        print("문장=%4d  fix=%d  위성=%2d  lat=%s lon=%s alt=%s  utc=%s"
              % (g.sentences, g.fix, g.sats, _fmt6(g.lat), _fmt6(g.lon),
                 _fmt(g.alt), g.utc))
        sleep_ms(1000)
    if g.sentences == 0:
        print("문장 0 -> 모듈 무응답. bench.gpsbaud() 로 보율부터 확인하세요.")
    elif g.fix == 0:
        print("문장은 오는데 픽스 없음 -> 모듈은 정상, 하늘이 보이는 곳에서 더 기다리세요.")


def _fmt6(v):
    return "--------" if v is None else "%.6f" % v


# ---------- 구동부 ----------
def servo(us):
    global _servo
    if _servo is None:
        _servo = PWM(Pin(config.SERVO_PIN))
        _servo.freq(config.SERVO_FREQ)
    us = int(us)
    if us < 400 or us > 2600:
        print("펄스 범위(400~2600us)를 벗어남:", us)
        return
    _servo.duty_ns(us * 1000)
    print("SERVO GP%d = %d us" % (config.SERVO_PIN, us))


def angle(deg):
    # 일반적인 500~2500us = 0~180도 가정. 기구에 맞춰 servo(us)로 미세조정 권장.
    if deg < 0:
        deg = 0
    elif deg > 180:
        deg = 180
    servo(500 + (deg / 180.0) * 2000.0)


def safe():
    servo(config.SERVO_SAFE_US)
    print("  -> 이 위치가 '낙하산 잠금'이어야 합니다")


def deploy():
    servo(config.SERVO_DEPLOY_US)
    print("  -> 이 위치가 '낙하산 사출'이어야 합니다")


def sweep(a=1000, b=2000, step=25, delay=60):
    print("%d -> %d -> %d us 왕복" % (a, b, a))
    for us in range(a, b + 1, step):
        servo(us)
        sleep_ms(delay)
    for us in range(b, a - 1, -step):
        servo(us)
        sleep_ms(delay)


def servo_off():
    global _servo
    if _servo:
        _servo.deinit()
        _servo = None
    print("서보 PWM 정지 (힘 빠짐)")


def buzzer(ms=1000):
    p = Pin(config.BUZZER_PIN, Pin.OUT)
    print("부저 ON %dms (GP%d)" % (ms, config.BUZZER_PIN))
    p.value(1)
    sleep_ms(ms)
    p.value(0)
    print("부저 OFF - 소리가 안 났으면 트랜지스터/배선 확인")


# ---------- 입력 ----------
def umbilical(n=60):
    from stage_io import StageInputs
    si = StageInputs()
    print("엄빌리컬을 하나씩 뽑아보세요. (연결=LOW=체결)")
    print("판정: 0=전부연결  1=SAFE만분리  2=전부분리  None=과도/불명")
    for _ in range(n):
        s, p1, p2 = si.raw_connected()
        print("SAFE(GP%d)=%s  POGO1(GP%d)=%s  POGO2(GP%d)=%s   -> 스테이지 %s"
              % (config.SAFETY_PIN, "체결" if s else "분리",
                 config.POGO1_PIN, "체결" if p1 else "분리",
                 config.POGO2_PIN, "체결" if p2 else "분리",
                 si.read()))
        sleep_ms(400)


print("bench 로드됨.  bench.help() 로 사용법 확인")