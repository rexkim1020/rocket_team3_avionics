# 지상 오검출 시험 - 트리거(3) 임계값 확정용.
#
#   비행 코드와 '똑같은' 파이프라인(자세 쿼터니언 -> 중력방향 투영 -> LPF ->
#   사다리꼴 적분 -> BNODeployLogic)을 그대로 돌리면서,
#     ㄱ) 가짜 신호가 어디까지 커지는지 통계를 재고
#     ㄴ) 실제 비행 판정 로직이 '오발동하는지'를 직접 확인한다.
#   서보는 절대 건드리지 않는다(측정 전용).
#
#   사용법 (Thonny Shell 또는 mpremote repl):
#       import falsetest
#       falsetest.run(30, "A")     # 30초, 시험이름 A
#
#   시험 목록은 falsetest.guide() 참고.
from machine import Pin, I2C
from time import sleep_ms, ticks_ms, ticks_diff
import math
import config
from bno055 import BNO055
from attitude import Attitude
from logic_bno import BNODeployLogic

_bno = None


def guide():
    print("""
=== 지상 오검출 시험 순서 ===
 각 시험은 따로 실행하고, 마지막 5초는 반드시 완전히 정지하세요.

  falsetest.run(30, "A")  완전정지        - 30초 가만히
  falsetest.run(30, "B")  직선 상하       - 위로 올렸다 내렸다
  falsetest.run(30, "C")  좌우흔들며 상승  - 흔들면서 위로
  falsetest.run(30, "D")  연직 S자        - 위아래 S자 반복 (가장 가혹)
  falsetest.run(30, "E")  회전만          - 제자리에서 자세만 회전

 ★ 위 시험들은 ①상승확인(5 m/s)에서 막혀 ②③④가 검증되지 않는다.
   전체 판정 체인을 검증하려면 chain() 을 쓸 것:
  falsetest.chain(30, "C2")   좌우흔들며 상승
  falsetest.chain(30, "D2")   연직 S자 (가장 가혹)

 팁: 실제 로켓처럼 세워서, 발사대 각도(약 85도)로 잡고 하면 더 정확합니다.
 각 시험이 끝나면 나오는 [RESULT] 블록을 그대로 복사해서 주세요.
""")


def chain(seconds=30, name="?", csv=False):
    # ②③④ 전체 체인 검증 전용. ①상승확인 임계를 자동으로 낮춰서 실행한다.
    # (run(..., arm_speed=...) 를 쓸 때 인자를 빠뜨리는 실수를 막기 위한 래퍼)
    return run(seconds, name, csv=csv, arm_speed=0.15)


def _sensor():
    global _bno
    if _bno is None:
        i2c = I2C(config.BNO_I2C_ID, sda=Pin(config.BNO_SDA_PIN),
                  scl=Pin(config.BNO_SCL_PIN), freq=config.I2C_FREQ)
        _bno = BNO055(i2c, config.IMU_AXIAL_AXIS)
    return _bno


def run(seconds=30, name="?", csv=False, arm_speed=None):
    # arm_speed: ①상승확인 임계를 이 시험에서만 낮춘다.
    #   손으로는 5 m/s 를 못 만들어서 그냥 두면 ①에서 막혀 ②③④가 검증되지 않는다.
    #   예: falsetest.run(30, "D2", arm_speed=0.15) 로 전체 체인을 강제로 통과시켜
    #   ②감속확인 ③게이트 ④음수속도 단계가 가짜신호로 뚫리는지 확인한다.
    #   config 원본은 건드리지 않고 시험이 끝나면 복원한다.
    b = _sensor()
    _saved_arm = config.IMU_MIN_SPEED
    if arm_speed is not None:
        config.IMU_MIN_SPEED = arm_speed
        print("=" * 52)
        print(" 체인검증 모드: IMU_MIN_SPEED %.1f -> %.2f (이 시험만)"
              % (_saved_arm, arm_speed))
        print("=" * 52)
    else:
        print("[일반 모드] IMU_MIN_SPEED = %.1f (②③④는 검증 안 됨)" % _saved_arm)
        print("   전체 체인을 보려면 falsetest.chain(30, \"%s\") 을 쓰세요" % name)

    # ---- 발사대 기준값 (비행 코드와 동일하게 3축 중력벡터 + 자이로 영점) ----
    print("기준값 측정 3초 - 움직이지 마세요...")
    aref = [0.0, 0.0, 0.0]
    bias = [0.0, 0.0, 0.0]
    n = 0
    for _ in range(150):
        try:
            a, w = b.read_accel_gyro()
            for k in range(3):
                aref[k] += a[k]
                bias[k] += w[k]
            n += 1
        except OSError:
            pass
        sleep_ms(20)
    aref = [v / n for v in aref]
    bias = [v / n for v in bias]
    g0 = math.sqrt(aref[0] ** 2 + aref[1] ** 2 + aref[2] ** 2)
    u = [v / g0 for v in aref]
    print("|g0| = %.4f   기울기기준 설정 완료" % g0)
    print(">>> 시험 [%s] 시작. %d초. 마지막 5초는 완전히 정지 <<<" % (name, seconds))

    att = Attitude(config.IMU_AXIAL_AXIS)
    logic = BNODeployLogic()
    t0 = ticks_ms()
    logic.reset(t0, g0)
    last = t0

    f = None
    if csv:
        try:
            f = open("/sd/falsetest_%s.csv" % name, "w")
            f.write("t_ms,dt_ms,vert_a,a_filt,v_imu,tilt\n")
        except Exception as e:
            print("[CSV] 저장 실패:", e)
            f = None

    # ---- 통계 ----
    v_max = -1e9        # Vfalse_pos
    v_min = 1e9         # Vfalse_neg
    a_min = 1e9         # Afalse_neg (LPF 후)
    dt_max = 0.0
    neg_start = None
    neg_hold_max = 0    # Tfalse_neg
    dec_start = None
    dec_hold_max = 0    # TAfalse_neg
    tilt_max = 0.0
    fired_at = None
    samples = 0

    t_end = ticks_ms() + seconds * 1000
    while ticks_diff(t_end, ticks_ms()) > 0:
        now = ticks_ms()
        dt = ticks_diff(now, last) / 1000.0
        last = now
        try:
            a, w = b.read_accel_gyro()
        except OSError:
            sleep_ms(20)
            continue
        att.update(w[0] - bias[0], w[1] - bias[1], w[2] - bias[2], dt)
        r = att.rotate(a[0], a[1], a[2])
        vert_a = r[0] * u[0] + r[1] * u[1] + r[2] * u[2]

        el = ticks_diff(now, t0)
        fire = logic.update(now, el, vert_a)
        if fire and fired_at is None:
            fired_at = el

        v = logic.velocity
        af = logic.a_filt
        tl = att.tilt_deg()
        samples += 1
        if dt * 1000.0 > dt_max:
            dt_max = dt * 1000.0
        if v > v_max:
            v_max = v
        if v < v_min:
            v_min = v
        if af < a_min:
            a_min = af
        if tl > tilt_max:
            tilt_max = tl

        # 음수속도 지속시간 (임계값 이하로 유지된 최장 구간)
        if v <= config.IMU_NEG_SPEED_MPS:
            if neg_start is None:
                neg_start = now
            elif ticks_diff(now, neg_start) > neg_hold_max:
                neg_hold_max = ticks_diff(now, neg_start)
        else:
            neg_start = None
        # 감속 지속시간
        if af <= config.IMU_DECEL_MPS2:
            if dec_start is None:
                dec_start = now
            elif ticks_diff(now, dec_start) > dec_hold_max:
                dec_hold_max = ticks_diff(now, dec_start)
        else:
            dec_start = None

        if f:
            try:
                f.write("%d,%.1f,%.3f,%.3f,%.3f,%.1f\n"
                        % (el, dt * 1000.0, vert_a, af, v, tl))
            except Exception:
                pass
        sleep_ms(20)

    if f:
        try:
            f.close()
        except Exception:
            pass

    print("")
    print("[RESULT] ---- 이 블록을 그대로 복사해서 주세요 ----")
    print("test         = %s" % name)
    print("duration_s   = %d" % seconds)
    print("samples      = %d" % samples)
    print("Vfalse_pos   = %+.3f m/s      (가짜 최대 양수속도)" % v_max)
    print("Vfalse_neg   = %+.3f m/s      (가짜 최대 음수속도)" % v_min)
    print("Tfalse_neg   = %d ms          (음수 %.1f 이하 최장 지속)"
          % (neg_hold_max, config.IMU_NEG_SPEED_MPS))
    print("Afalse_neg   = %+.3f m/s^2    (LPF 후 가장 음수인 가속)" % a_min)
    print("TAfalse_neg  = %d ms          (감속 %.1f 이하 최장 지속)"
          % (dec_hold_max, config.IMU_DECEL_MPS2))
    print("dt_max       = %.1f ms" % dt_max)
    print("tilt_max     = %.1f deg" % tilt_max)
    print("v_end        = %+.3f m/s      (종료 시 잔류속도)" % logic.velocity)
    print("armed        = %s   coasting = %s   gate = %s"
          % (logic.armed, logic.coasting, logic.gate_open))
    if fired_at is None:
        print("FIRED        = NO            <-- 오발동 없음 (좋음)")
    else:
        print("FIRED        = YES at %d ms  <-- ★오발동! 임계값 조정 필요" % fired_at)
    print("thresholds   = MIN_SPEED %.2f / DECEL %.1f hold %d / GUARD %d / NEG %.1f hold %d / LPF %.0fHz"
          % (config.IMU_MIN_SPEED, config.IMU_DECEL_MPS2, config.IMU_DECEL_HOLD_MS,
             config.EARLY_GUARD_MS, config.IMU_NEG_SPEED_MPS,
             config.IMU_NEG_HOLD_MS, config.IMU_ACCEL_LPF_HZ))
    if not logic.armed:
        print("주의: armed=False -> ①에서 막혀 ②③④는 검증되지 않았음.")
        print("      arm_speed=0.15 로 다시 돌리면 전체 체인을 검증할 수 있음.")
    print("[/RESULT] ------------------------------------------")
    config.IMU_MIN_SPEED = _saved_arm
