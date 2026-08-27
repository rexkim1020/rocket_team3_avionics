# ============================================================
#  GoRocket 단일단 로켓 메인 (MicroPython / Raspberry Pi Pico)
#  임무: 낙하산 사출 + SD 로깅 + GPS 회수 (무선 없음)
#
#  [간소화 버전]
#    - 비행 중 재부팅 복구(SD 상태저장) 없음 : 배터리 커넥터가 단단하고
#      커패시터가 있어 비행 중 전원이 끊기지 않는다는 전제.
#      -> 대신 부팅 분기가 하나뿐이라 구조가 단순하고, 이전 비행기록 때문에
#         전원 켜자마자 서보가 사출 위치로 가던 문제도 없다.
#    - 워치독(WDT)은 유지 : '발사 전 멈춤'을 구제한다. 부저가 타이머로 돌기
#      때문에 루프가 멈춰도 소리는 계속 나서 사람이 눈치챌 수 없기 때문.
#      (비행 중 리셋은 구제 못 함 - 복구 로직이 없어 상태를 모른다)
#    그 외(스테이지·부저·사출판정)는 원래대로.
#
#  상태: Stage0 대기 <-> Stage1 무장 -> Stage2 발사 -> Stage3 사출
#  사출: (1)기압계 정점  OR  (2)백업  OR  (3)IMU 연직속도  (T+SAFE_TIME_MS 이후)
#
#  [안전 규칙] 발사 인정 조건 = "부팅 후 엄빌리컬이 연결된 상태를 본 적이 있을 것"
#     -> 엄빌 안 꽂고 켜면 발사로 인정하지 않는다(벤치 오사출 방지).
# ============================================================
import math
from machine import Pin, I2C, WDT
from time import ticks_ms, ticks_diff, ticks_add, sleep_ms

import config
from bmp388 import BMP388
from bno055 import BNO055
from attitude import Attitude
from stage_io import StageInputs
from buzzer import Buzzer
from servo_release import ParachuteRelease
from gps import GPS
from sdlog import SDLog
from velocity import BaroVelocity
from logic_bmp import BMPDeployLogic
from logic_bno import BNODeployLogic
from logic_backup import BackupDeploy


def altitude_of(pressure, reference):
    if reference <= 0.0:
        return 0.0
    return 44330.0 * (1.0 - math.pow(pressure / reference, 0.19029495718))


def _f(x, nd=3):
    if x is None:
        return ""
    if nd == 6:
        return "%.6f" % x
    elif nd == 2:
        return "%.2f" % x
    elif nd == 1:
        return "%.1f" % x
    return "%.3f" % x


# ============================================================
#  초기화
# ============================================================
print("[BOOT] GoRocket single-stage (간소화판)")

release = ParachuteRelease()          # 서보를 가장 먼저 잠금 위치로
buzzer = Buzzer()
buzzer.start_timer(config.BUZZER_TIMER_MS)   # 부저는 타이머로 재생(리듬 일정)
inputs = StageInputs()

i2c_bmp = I2C(config.BMP_I2C_ID, sda=Pin(config.BMP_SDA_PIN),
              scl=Pin(config.BMP_SCL_PIN), freq=config.I2C_FREQ)
i2c_bno = I2C(config.BNO_I2C_ID, sda=Pin(config.BNO_SDA_PIN),
              scl=Pin(config.BNO_SCL_PIN), freq=config.I2C_FREQ)

bmp = None
try:
    bmp = BMP388(i2c_bmp)
    print("[OK] BMP388 @", hex(bmp.address))
except Exception as e:
    print("[FAIL] BMP388:", e)

bno = None
try:
    bno = BNO055(i2c_bno, config.IMU_AXIAL_AXIS)
    print("[OK] BNO055 @", hex(bno.address),
          "6면보정 적용" if bno.HAS_CAL else "6면보정 없음")
except Exception as e:
    print("[FAIL] BNO055:", e)

gps = GPS()
sd = SDLog()
#  로그는 항상 이어붙인다(append). 전원을 껐다 켜도 flight.csv 뒤에 계속 쌓이므로
#  SD 를 포맷할 필요가 없다. 대신 부팅마다 구분선을 넣어 세션을 나눈다.
if sd.ok:
    sd.event("=" * 20 + " BOOT " + "=" * 20)
    sd.flush_ms = config.SD_PAD_FLUSH_MS   # 발사 전에는 커밋도 뜸하게
att = Attitude(config.IMU_AXIAL_AXIS)
baro_vel = BaroVelocity()
bmp_logic = BMPDeployLogic()
bno_logic = BNODeployLogic()
backup_logic = BackupDeploy()

# ---------- 기준값 (발사대에서 계속 갱신하다 발사 순간 고정) ----------
#  중력 방향은 3축 가속도 평균으로 구한다. 축 하나만 쓰면 X/Y 바이어스와
#  발사대 기울기가 보정되지 않는다.
reference_pressure = 101325.0
gyro_bias = [0.0, 0.0, 0.0]
accel_ref = [0.0, 0.0, 0.0]
grav_u = [0.0, 0.0, 0.0]
grav_u[config.IMU_AXIAL_AXIS] = config.IMU_AXIAL_SIGN
ground_accel = 9.80665


def refresh_gravity_ref():
    global ground_accel
    m = math.sqrt(accel_ref[0] ** 2 + accel_ref[1] ** 2 + accel_ref[2] ** 2)
    if m > 1.0:
        grav_u[0] = accel_ref[0] / m
        grav_u[1] = accel_ref[1] / m
        grav_u[2] = accel_ref[2] / m
        ground_accel = m


if bmp:
    try:
        reference_pressure = bmp.read()[0]
    except Exception:
        pass
if bno:
    try:
        _a, _w = bno.read_accel_gyro()
        accel_ref = [_a[0], _a[1], _a[2]]
        gyro_bias = [_w[0], _w[1], _w[2]]
        refresh_gravity_ref()
    except Exception:
        pass

# ============================================================
#  부팅 판정 + 자가진단(POST)
#    전부 정상 -> "삐— 삐—"(길게 2번) / 하나라도 문제 -> "삐삐삐삐삐"(짧게 5번)
#    GPS 위성고정은 수십 초~수 분 걸리므로 부팅을 막지 않는다. 센서가 정상이면
#    판정을 보류했다가 픽스가 잡히는 순간 OK 를 울린다.
#    => "삐— 삐—" 가 곧 "이제 발사해도 된다" 는 신호.
# ============================================================
sc, p1c, p2c = inputs.raw_connected()
armed_seen = not ((not sc) and (not p1c) and (not p2c))
if armed_seen:
    print("[BOOT] 엄빌리컬 연결 확인 -> 정상 시작")
else:
    print("[BOOT] 엄빌리컬 미체결로 부팅 -> 발사불가(IDLE)")
    print("       한 번 꽂았다 빼야 발사로 인정합니다")

# BNO 는 객체 생성만으로 부족하다. 0만 내보내는 고장을 값으로 확인한다.
bno_alive = (bno is not None) and bno.is_alive()
sd_ok_for_post = sd.ok or (not config.SD_REQUIRED)
sensors_ok = (bmp is not None) and bno_alive and sd_ok_for_post

_t0 = ticks_ms()
while ticks_diff(ticks_ms(), _t0) < config.GPS_POST_MS:
    gps.update()
    if gps.sentences > 0:            # NMEA 한 문장이라도 오면 모듈은 살아있음
        break
    sleep_ms(20)

post_verdict = None                  # None=픽스 대기중, True=OK, False=FAIL
post_mode = None                     # 지금 낼 결과음 이름
post_until = ticks_ms()
boot_ms = ticks_ms()

#  ★ 결과음 3단계 — 센서 통과를 즉시 알려야 한다.
#    센서만 보고 끝내지 않고 GPS 픽스까지 기다리면, 실내처럼 위성을 못 잡는
#    곳에서는 몇 분이고 대기음만 나서 "진단이 통과한 건지 멈춘 건지" 알 수 없다.
#      삐삐삐삐삐 (짧게 5번) = 센서 고장
#      삐———     (길게 1번) = 센서 정상, GPS 위성 대기중
#      삐— 삐—   (길게 2번) = 전부 준비 완료 (발사 가능)
#  ★ GPS 무응답은 여기서 실패로 확정하지 않는다(전원 순서 의존성 제거).
#    5V 가 3.7V 보다 늦게 켜지면 부팅 시점엔 GPS 가 무전원인 게 정상이다.
_fails = []
if bmp is None:        _fails.append("BMP388")
if bno is None:        _fails.append("BNO055")
elif not bno_alive:    _fails.append("BNO055(값이 0)")
if not sd.ok:
    if config.SD_REQUIRED:
        _fails.append("SD")
    else:
        print("[POST] 경고: SD 없음 - 로깅 없이 진행 (SD_REQUIRED=False)")
if _fails:
    print("[POST] FAIL:", ", ".join(_fails))
    post_verdict = False
    post_mode = "postfail"
else:
    print("[POST] 센서 정상 -> 삐———  (GPS 위성 %s)"
          % ("모듈 응답 대기중" if gps.sentences == 0 else "고정 대기중"))
    post_mode = "sensorok"
post_until = ticks_add(ticks_ms(), config.POST_SOUND_MS)

# ============================================================
#  상태
# ============================================================
state = 0
launch_ms = None
deployed = False
pressure = None
temp = None
altitude = None
v_baro = None
gps_fix = False           # 안정화(디바운스)된 픽스 상태
gps_fix_since = None      # 상태가 바뀌려고 한 시각
gps_fix_prev = False      # 획득/상실 순간 출력용
vel_bad_since = None
bno_zero = 0
bno_retry = 0
loop_err = 0
last_loop = ticks_ms()
gps_print = ticks_ms()
last_log = ticks_ms()     # 발사 전 저속 로깅(SD 노이즈로 GPS 방해 방지)용

# 워치독: 발사 전 멈춤을 구제한다. 비행 중 리셋은 구제 못 함(복구 로직 없음).
wdt = WDT(timeout=config.WDT_MS) if config.WDT_ENABLED else None
if wdt is None:
    print("[WARN] 워치독 비활성 (벤치용). 비행 전 config.WDT_ENABLED=True 확인!")

print("[RUN] 루프 시작. stage =", state, " armed_seen =", armed_seen)
print("      (재부팅 복구 없음 - 간소화판 / 워치독 %s)"
      % ("ON" if wdt else "OFF"))

# ============================================================
#  메인 루프
#    루프 전체를 try 로 감싼다. 센서 오류 한 번으로 사출을 못 하면 안 된다.
# ============================================================
while True:
    try:
        now = ticks_ms()
        dt = ticks_diff(now, last_loop) / 1000.0
        last_loop = now
        gps.update()

        # ---------- 센서 ----------
        ax = ay = az = wx = wy = wz = vert_a = tilt = None
        bmp_fresh = False
        if bmp:
            try:
                # 새 변환이 끝난 샘플일 때만 로직에 넣는다. 같은 값을 중복
                # 처리하면 median/이동평균/하강확인 횟수가 왜곡된다.
                _pt = bmp.read_if_ready()
                if _pt is not None:
                    pressure, temp = _pt
                    altitude = altitude_of(pressure, reference_pressure)
                    v_baro = baro_vel.update(now, altitude)
                    bmp_fresh = True
            except Exception:
                pass
        if bno:
            try:
                _a, _w = bno.read_accel_gyro()
                ax, ay, az = _a
                wx = _w[0] - gyro_bias[0]
                wy = _w[1] - gyro_bias[1]
                wz = _w[2] - gyro_bias[2]
                if state >= 2:
                    att.update(wx, wy, wz, dt)      # 비행 중: 자이로 적분
                    tilt = att.tilt_deg()
                    _r = att.rotate(ax, ay, az)     # 기준계로 회전
                else:
                    tilt = bno.tilt_from_accel(_a)  # 발사 전: 중력 기준
                    _r = (ax, ay, az)
                # 발사대에서 실측한 중력 방향에 투영 -> '연직' 성분
                vert_a = _r[0] * grav_u[0] + _r[1] * grav_u[1] + _r[2] * grav_u[2]
            except Exception:
                pass

        # ---------- BNO 가 0만 내보내면 발사 전에 한해 재초기화 ----------
        # 파워온 타이밍이 나쁘면 초기화가 성공한 듯 보이면서 계속 0만 나온다.
        # 재초기화는 약 0.8초 블로킹이라 비행 중에는 절대 하지 않는다.
        if bno is not None and state < 2 and ax is not None:
            if abs(ax) + abs(ay) + abs(az) < 0.05:
                bno_zero += 1
                if bno_zero >= config.BNO_ZERO_REINIT and bno_retry < config.BNO_REINIT_MAX:
                    bno_retry += 1
                    bno_zero = 0
                    print("[BNO] 0값 지속 -> 재초기화 #%d" % bno_retry)
                    try:
                        bno = BNO055(i2c_bno, config.IMU_AXIAL_AXIS)
                        bno_alive = bno.is_alive()
                        sensors_ok = (bmp is not None) and bno_alive and sd_ok_for_post
                        print("[BNO] 재초기화 %s" % ("성공" if bno_alive else "여전히 0"))
                    except Exception as e:
                        print("[BNO] 재초기화 실패:", e)
            else:
                bno_zero = 0

        # ---------- GPS 픽스 안정화 ----------
        # 위성 경계에 걸치면 fix 가 초 단위로 깜빡인다. 그대로 쓰면 준비완료음이
        # 왔다갔다 해서 발사장에서 판단이 불가능하다. 일정 시간 유지돼야 바꾼다.
        _fix_raw = gps.fix > 0
        if _fix_raw != gps_fix:
            if gps_fix_since is None:
                gps_fix_since = now
            elif ticks_diff(now, gps_fix_since) >= config.GPS_FIX_DEBOUNCE_MS:
                gps_fix = _fix_raw
                gps_fix_since = None
        else:
            gps_fix_since = None
        ready = sensors_ok and gps_fix

        # ---------- POST 최종 판정 (GPS 위성 고정을 기다렸다 확정) ----------
        if post_verdict is None and state < 2:
            if gps_fix:
                post_verdict = True
                post_mode = "postok"
                post_until = ticks_add(now, config.POST_SOUND_MS)
                print("[POST] ALL OK - 위성 고정 sats=%d lat=%s lon=%s"
                      % (gps.sats, _f(gps.lat, 6), _f(gps.lon, 6)))
            elif (gps.sentences == 0
                    and ticks_diff(now, boot_ms) >= config.POST_GPS_NMEA_WAIT_MS):
                # 5V 가 늦게 켜지는 경우까지 기다려 준 뒤에도 무응답이면 진짜 고장
                post_verdict = False
                post_mode = "postfail"
                post_until = ticks_add(now, config.POST_SOUND_MS)
                print("[POST] FAIL: GPS 모듈 무응답 (%d초 대기) - 5V/배선 확인"
                      % (config.POST_GPS_NMEA_WAIT_MS // 1000))
            elif ticks_diff(now, boot_ms) >= config.POST_GPS_FIX_WAIT_MS:
                post_verdict = False
                post_mode = "postfail"
                post_until = ticks_add(now, config.POST_SOUND_MS)
                print("[POST] FAIL: GPS 위성 고정 실패 (문장=%d sats=%d)"
                      % (gps.sentences, gps.sats))

        if state < 2:
            if gps_fix != gps_fix_prev:
                print("[GPS] %s (sats=%d)"
                      % ("위성 고정" if gps_fix else "픽스 상실", gps.sats))
            if ticks_diff(now, gps_print) >= 5000:
                print("[GPS] fix=%d sats=%d 문장=%d" % (gps.fix, gps.sats, gps.sentences))
                gps_print = now
        gps_fix_prev = gps_fix

        # ---------- 상태머신 (0<->1 가역, ->2 래치) ----------
        raw = inputs.read()
        if state < 2 and raw is not None:
            if raw == 2:
                if armed_seen:                    # ★ 안전 규칙
                    launch_ms = now
                    state = 2
                    bmp_logic.reset()
                    bno_logic.reset(now, ground_accel)
                    backup_logic.reset()
                    att.reset()                   # 발사 순간 = tilt 0도 기준
                    baro_vel.reset()
                    if sd.ok:
                        # 발사했으니 이제 전체 속도로 기록한다
                        sd.flush_ms = config.SD_FLUSH_MS
                        sd.event("LAUNCH t=%d ref_p=%.1f g0=%.3f"
                                 % (now, reference_pressure, ground_accel))
                    print("[LAUNCH] t=", now)
            else:
                armed_seen = True                 # 연결된 상태를 실제로 관측
                if raw == 1:
                    if not (config.REQUIRE_GPS_FIX_TO_ARM and not gps_fix):
                        state = 1
                else:
                    state = 0

        # ---------- 발사 전: 기준값 갱신 ----------
        if state < 2:
            if bmp_fresh and pressure is not None:
                reference_pressure = 0.98 * reference_pressure + 0.02 * pressure
            if ax is not None:
                accel_ref[0] = 0.98 * accel_ref[0] + 0.02 * ax
                accel_ref[1] = 0.98 * accel_ref[1] + 0.02 * ay
                accel_ref[2] = 0.98 * accel_ref[2] + 0.02 * az
                refresh_gravity_ref()
            if wx is not None:
                k = config.GYRO_BIAS_ALPHA
                gyro_bias[0] += k * wx
                gyro_bias[1] += k * wy
                gyro_bias[2] += k * wz

        flight_ms = 0
        if state >= 2 and launch_ms is not None:
            flight_ms = ticks_diff(now, launch_ms)

        # ---------- IMU 속도 적분 ----------
        # 사출 후(state 3)에도 계속 돌린다. 판정에는 안 쓰지만 로그에 낙하산
        # 하강률이 남는다. (안 돌리면 vel_imu 가 사출 시점 값으로 얼어붙는다)
        imu_fire = False
        if state >= 2 and vert_a is not None:
            imu_fire = bno_logic.update(now, flight_ms, vert_a)

        # ---------- 속도 교차검증 (기압 미분 vs IMU 적분) ----------
        # 크게 어긋난 상태가 지속되면 적분이 드리프트한 것이므로 IMU 트리거를
        # 끈다. 기압계(1)와 백업(2)은 그대로 살아있다.
        if (config.VEL_CROSSCHECK and state == 2 and bno_logic.saturated is False
                and v_baro is not None and flight_ms >= config.VEL_CHECK_AFTER_MS):
            if abs(bno_logic.velocity - v_baro) > config.VEL_DISAGREE_MPS:
                if vel_bad_since is None:
                    vel_bad_since = now
                elif ticks_diff(now, vel_bad_since) >= config.VEL_DISAGREE_MS:
                    bno_logic.saturated = True    # 트리거(3) 무효화
                    if sd.ok:
                        sd.event("VEL_MISMATCH imu=%.1f baro=%.1f"
                                 % (bno_logic.velocity, v_baro))
                    print("[VEL] 불일치로 IMU 트리거 해제")
            else:
                vel_bad_since = None

        # ---------- 사출 판정 ----------
        if state == 2 and launch_ms is not None:
            bmp_fire = config.DEPLOY_USE_BARO and bmp_fresh and altitude is not None \
                and bmp_logic.update(altitude)
            bno_fire = config.DEPLOY_USE_IMU and imu_fire
            backup_fire = config.DEPLOY_USE_BACKUP \
                and backup_logic.update(flight_ms, altitude)

            if flight_ms >= config.SAFE_TIME_MS and (bmp_fire or bno_fire or backup_fire):
                release.deploy(now)
                deployed = True
                state = 3
                reason = "BARO" if bmp_fire else ("IMU" if bno_fire else "BACKUP")
                if sd.ok:
                    sd.event("DEPLOY reason=%s flight_ms=%d alt=%s tilt=%s"
                             % (reason, flight_ms, _f(altitude, 1), _f(tilt, 1)))
                print("[DEPLOY]", reason, "flight_ms=", flight_ms)

        # ---------- 서보: 사출 후 잠금 각 복귀 ----------
        if release.update(now):
            print("[SERVO] 사출 %d초 경과 -> 잠금 각 복귀 (전류 절약)"
                  % (config.SERVO_RELOCK_MS // 1000))
            if sd.ok:
                sd.event("SERVO_RELOCK flight_ms=%d" % flight_ms)

        # ---------- 부저 ----------
        # 로깅보다 먼저 한다. 뒤에서 예외가 나도 소리는 현재 상태를 따라가야 한다.
        if state == 3:
            mode = "beacon" if config.RECOVERY_BEACON else "silent"
        elif state == 2:
            mode = "silent"
        elif post_mode is not None and ticks_diff(post_until, now) > 0:
            mode = post_mode                # 자가진단 결과음 (sensorok/postok/postfail)
        elif not armed_seen:
            mode = "idle"
        elif state == 1:
            mode = "ready" if ready else "wait"
        else:
            mode = "s0"
        buzzer.set(mode)

        # ---------- 로깅 ----------
        # ★ 발사 전에는 초당 1행만 쓴다. SD SPI 노이즈가 GPS 위성 획득을
        #   방해하기 때문(실측 확인됨). 발사 후에는 매 루프 전체 기록.
        if state >= 2 or ticks_diff(now, last_log) >= config.SD_PAD_LOG_MS:
            last_log = now
            vel = bno_logic.velocity if state >= 2 else 0.0
            sd.write(",".join((
                "%d" % now, "%d" % state, "%d" % flight_ms,
                _f(altitude, 2), _f(pressure, 1), _f(temp, 2),
                _f(ax), _f(ay), _f(az), _f(wx), _f(wy), _f(wz),
                _f(vert_a), _f(vel, 3), _f(v_baro, 3), _f(tilt, 2),
                "%d" % gps.fix, "%d" % gps.sats,
                _f(gps.lat, 6), _f(gps.lon, 6), _f(gps.alt, 1), gps.utc,
                "1" if deployed else "0",
            )) + "\n")

        buzzer.update()
        loop_err = 0
    except Exception as e:
        loop_err += 1
        if loop_err <= 3 or loop_err % 50 == 0:
            print("[LOOP ERR] #%d %s" % (loop_err, e))

    # try 밖에서 먹인다. 잡힌 예외는 '멈춤'이 아니므로 리셋시키지 않는다.
    if wdt:
        wdt.feed()
    sleep_ms(config.LOOP_MS)
