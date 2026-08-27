# 지상 리허설 - 부저 소리와 상태 전환을 실제와 똑같이 확인한다.
#
#   비행 코드(main.py)와 같은 모듈(Buzzer / StageInputs / POST 절차)을 그대로
#   쓰므로 소리와 상태 흐름은 실제 비행과 동일하다.
#   다만 사출 판정(기압계/IMU 정점감지)은 지상에서 재현할 수 없으므로,
#   "발사 후 N초" 로 사출을 흉내내서 Stage3(착륙 비콘)까지 들려준다.
#
#   ★ 서보는 기본적으로 건드리지 않는다(사출 장치 미완성 대비).
#     servo=True 로 줘야 실제로 움직인다.
#
#   사용법 (Thonny Shell / mpremote repl):
#       import groundtest
#       groundtest.sounds()        # 소리만 빠르게 순서대로 듣기
#       groundtest.run()           # 엄빌리컬 꽂았다 뽑으며 실제 리허설
#
#   빠져나오려면 Ctrl-C.
from machine import Pin, I2C
from time import sleep_ms, ticks_ms, ticks_diff
import config
from buzzer import Buzzer
from stage_io import StageInputs


def sounds(each=5000):
    # 각 단계 소리를 순서대로 재생. 스피커/트랜지스터 확인과 패턴 익히기용.
    bz = Buzzer()
    print("=== 부저 패턴 확인 (각 %.1f초) ===" % (each / 1000.0))

    print("\n[자가진단 정상] 삐— 삐—  (센서 3종 + GPS 위성고정 전부 OK)")
    bz.post_ok()
    sleep_ms(800)

    print("[자가진단 실패] 삐삐삐삐삐  (하나라도 문제)")
    bz.post_fail(1)
    sleep_ms(600)

    for mode, label in (("s0", "Stage0 대기 (삐- 삐-)"),
                        ("wait", "Stage1 준비안됨 (빠른 삐삐삐)"),
                        ("ready", "Stage1 준비완료 (삐빅-)"),
                        ("idle", "발사불가 IDLE (2.4초마다 짧게)"),
                        ("beacon", "Stage3 착륙비콘 (삐— 삐—)")):
        print("\n[%s]" % label)
        bz.set(mode)
        t_end = ticks_ms() + each
        while ticks_diff(t_end, ticks_ms()) > 0:
            bz.update()
            sleep_ms(20)
    bz.set("silent")
    bz.update()
    print("\n=== 끝 (Stage2 발사중은 무음) ===")


def _post(bz, inputs, quick):
    # main.py 와 같은 자가진단. quick=True 면 센서 초기화를 건너뛰고 결과만 흉내.
    if quick:
        print("[POST] 건너뜀 (quick=True)")
        return True
    ok = True
    fails = []
    try:
        from bmp388 import BMP388
        BMP388(I2C(config.BMP_I2C_ID, sda=Pin(config.BMP_SDA_PIN),
                   scl=Pin(config.BMP_SCL_PIN), freq=config.I2C_FREQ))
        print("[OK] BMP388")
    except Exception as e:
        print("[FAIL] BMP388:", e)
        fails.append(("BMP388", 2))
        ok = False
    try:
        from bno055 import BNO055
        BNO055(I2C(config.BNO_I2C_ID, sda=Pin(config.BNO_SDA_PIN),
                   scl=Pin(config.BNO_SCL_PIN), freq=config.I2C_FREQ),
               config.IMU_AXIAL_AXIS)
        print("[OK] BNO055")
    except Exception as e:
        print("[FAIL] BNO055:", e)
        fails.append(("BNO055", 3))
        ok = False
    try:
        from sdlog import SDLog
        sd = SDLog()
        if not sd.ok:
            raise OSError("mount fail")
        print("[OK] SD")
    except Exception as e:
        print("[FAIL] SD:", e)
        fails.append(("SD", 4))
        ok = False

    if not fails:
        print("[POST] 센서 정상 -> 삐— 삐—")
        print("       (실제 main.py 는 GPS 위성고정까지 확인한 뒤 이 소리를 냅니다)")
        bz.post_ok()
    else:
        print("[POST] 실패:", ", ".join(n for n, _c in fails), "-> 삐삐삐삐삐")
        bz.post_fail()
    return ok


def run(servo=False, deploy_after_ms=8000, fake_gps_fix=True, quick=False):
    """엄빌리컬을 실제로 꽂았다 뽑으며 상태 전환과 소리를 확인한다.

    servo          : True 면 서보도 실제로 움직인다 (기본 False = 안 건드림)
    deploy_after_ms: 발사 인식 후 이 시간 뒤에 '사출'을 흉내낸다
    fake_gps_fix   : 실내에서 GPS 픽스가 없어도 Stage1 준비완료음을 들으려면 True
    quick          : True 면 POST 를 건너뛴다
    """
    bz = Buzzer()
    inputs = StageInputs()
    release = None
    if servo:
        from servo_release import ParachuteRelease
        release = ParachuteRelease()
        print("[서보] 활성 - 잠금 위치로 이동함")
    else:
        print("[서보] 비활성 (servo=True 로 주면 실제로 움직임)")

    print("")
    sensors_ok = _post(bz, inputs, quick)
    print("")
    print("=== 리허설 시작 ===")
    print(" 엄빌리컬 3개를 다 꽂으면 Stage0, 안전핀(GP%d)만 빼면 Stage1,"
          % config.SAFETY_PIN)
    print(" 3개 다 빼면 Stage2(발사) -> %.1f초 뒤 사출 흉내 -> Stage3 비콘"
          % (deploy_after_ms / 1000.0))
    print(" Ctrl-C 로 종료")
    print("")

    # main.py 와 동일한 안전 규칙: 연결된 상태를 본 적이 있어야 발사로 인정
    armed_seen = False
    sc, p1c, p2c = inputs.raw_connected()
    if not ((not sc) and (not p1c) and (not p2c)):
        armed_seen = True
        print("[BOOT] 엄빌리컬 연결 확인됨 -> 정상 시작")
    else:
        print("[BOOT] 엄빌리컬 전부 분리 상태로 시작 -> 발사불가(IDLE)")
        print("       한 번 꽂았다 빼야 발사로 인정합니다")

    state = 0
    launch_ms = None
    deployed = False
    last_print = ticks_ms()
    last_state = -1

    try:
        while True:
            now = ticks_ms()
            raw = inputs.read()

            if state < 2 and raw is not None:
                if raw == 2:
                    if armed_seen:
                        state = 2
                        launch_ms = now
                        print("\n>>> [Stage2] 발사 인식! 부저 무음. %.1f초 뒤 사출 흉내"
                              % (deploy_after_ms / 1000.0))
                else:
                    armed_seen = True
                    state = 1 if raw == 1 else 0

            if state == 2 and ticks_diff(now, launch_ms) >= deploy_after_ms:
                state = 3
                deployed = True
                if release:
                    release.deploy(now)
                print("\n>>> [Stage3] 사출! %s -> 착륙 비콘 시작"
                      % ("서보 동작함" if release else "서보 미동작(servo=False)"))

            # --- 사출 후 잠금 각 복귀 (main.py 와 동일) ---
            if release and release.update(now):
                print(">>> [서보] 사출 %d초 경과 -> 잠금 각 복귀"
                      % (config.SERVO_RELOCK_MS // 1000))

            # --- main.py 와 동일한 부저 모드 선택 ---
            ready = sensors_ok and fake_gps_fix
            if state == 3:
                mode = "beacon" if config.RECOVERY_BEACON else "silent"
            elif state == 2:
                mode = "silent"
            elif not armed_seen:
                mode = "idle"
            elif state == 1:
                mode = "ready" if ready else "wait"
            else:
                mode = "s0"
            bz.set(mode)
            bz.update()

            if state != last_state:
                names = ("Stage0 대기", "Stage1 무장", "Stage2 발사", "Stage3 사출후")
                print("  -> %s   (부저: %s)" % (names[state], mode))
                last_state = state

            if ticks_diff(now, last_print) >= 2000:
                s, p1, p2 = inputs.raw_connected()
                el = "" if launch_ms is None else "  T+%.1fs" % (
                    ticks_diff(now, launch_ms) / 1000.0)
                print("   SAFE %s  POGO1 %s  POGO2 %s   부저=%-6s%s"
                      % ("O" if s else "X", "O" if p1 else "X",
                         "O" if p2 else "X", mode, el))
                last_print = now

            sleep_ms(20)
    except KeyboardInterrupt:
        bz.set("silent")
        bz.update()
        print("\n[종료] 부저 정지")
        if release:
            release.safe()
            print("[서보] 잠금 위치로 복귀")
