# 서보 잠금/사출 각도 찾기 - 사출 장치 조립 후 1회.
#
#   ★ 안전 설계
#     - 항상 중립(1500us)에서 시작한다. config 의 임시값(1000/2000)으로
#       바로 가지 않는다. 기구 가동범위를 모르는 채 끝단으로 보내면
#       서보가 물려서(스톨) 과전류가 흐르고 기구가 상한다.
#     - 목표 위치로 한 번에 점프하지 않고 25us 씩 램프로 이동한다.
#     - 700~2300us 를 넘으면 경고한다.
#
#   사용법 (Thonny Shell / mpremote repl):
#       import servocal
#       servocal.start()          # 중립 1500us 에서 시작
#       servocal.step(-25)        # 25us 씩 움직이며 위치 찾기 (+/- 부호로 방향)
#       servocal.lock()           # 지금 위치를 '잠금'으로 기록
#       servocal.step(+300)       # 반대편으로 이동
#       servocal.release()        # 지금 위치를 '사출'로 기록
#       servocal.test(3)          # 잠금<->사출 3회 왕복하며 재현성 확인
#       servocal.show()           # config 에 넣을 값 출력
#       servocal.off()            # 서보 힘 빼기
from machine import Pin, PWM
from time import sleep_ms
import config

_pwm = None
_cur = None
_lock_us = None
_release_us = None

MIN_US = 500     # 0도
MAX_US = 2500    # 180도 (표준 서보 가동범위의 끝)
NEUTRAL = 1500


def start(us=NEUTRAL):
    global _pwm, _cur
    if _pwm is None:
        _pwm = PWM(Pin(config.SERVO_PIN))
        _pwm.freq(config.SERVO_FREQ)
    _cur = int(us)
    _pwm.duty_ns(_cur * 1000)
    print("서보 GP%d = %d us (중립에서 시작)" % (config.SERVO_PIN, _cur))
    print("  step(+25) / step(-25) 로 조금씩 움직이세요")
    print("  기구가 뻑뻑하거나 소리가 나면 즉시 반대로 되돌리세요")


def _goto(target):
    # 한 번에 점프하지 않고 25us 씩 램프 -> 기구에 충격을 주지 않는다
    global _cur
    if _pwm is None:
        print("먼저 servocal.start() 를 실행하세요")
        return
    target = int(target)
    if target < MIN_US or target > MAX_US:
        print("! %d us 는 안전범위(%d~%d)를 벗어납니다. 중단."
              % (target, MIN_US, MAX_US))
        return
    stepv = 25 if target > _cur else -25
    while _cur != target:
        nxt = _cur + stepv
        if (stepv > 0 and nxt > target) or (stepv < 0 and nxt < target):
            nxt = target
        _cur = nxt
        _pwm.duty_ns(_cur * 1000)
        sleep_ms(40)


def step(delta):
    _goto((_cur if _cur is not None else NEUTRAL) + delta)
    print("현재 %d us" % _cur)


def us(value):
    _goto(value)
    print("현재 %d us" % _cur)


def angle(deg):
    # 참고용 환산 (500~2500us = 0~180도 가정). 기구 실측은 us 로 하세요.
    if deg < 0:
        deg = 0
    elif deg > 180:
        deg = 180
    us(500 + (deg / 180.0) * 2000.0)


def lock():
    global _lock_us
    _lock_us = _cur
    print("잠금 위치 기록: %d us" % _lock_us)


def release():
    global _release_us
    _release_us = _cur
    print("사출 위치 기록: %d us" % _release_us)


def test(times=3, hold_ms=1200):
    # 기록한 두 위치를 왕복하며 재현성 확인. 낙하산은 빼고 하세요.
    if _lock_us is None or _release_us is None:
        print("먼저 lock() 과 release() 로 두 위치를 기록하세요")
        return
    for i in range(times):
        print("  %d/%d  잠금 %d us" % (i + 1, times, _lock_us))
        _goto(_lock_us)
        sleep_ms(hold_ms)
        print("  %d/%d  사출 %d us" % (i + 1, times, _release_us))
        _goto(_release_us)
        sleep_ms(hold_ms)
    _goto(_lock_us)
    print("잠금 위치로 복귀. 매번 같은 자리로 갔으면 정상입니다.")


def show():
    if _lock_us is None or _release_us is None:
        print("아직 기록 안 됨 (lock %s / release %s)" % (_lock_us, _release_us))
        return
    print("")
    print("=== config.py 에 아래처럼 넣으세요 ===")
    print("SERVO_SAFE_US   = %d" % _lock_us)
    print("SERVO_DEPLOY_US = %d" % _release_us)
    print("")
    print("이동량 %d us (%.0f도 상당)"
          % (abs(_release_us - _lock_us), abs(_release_us - _lock_us) * 180.0 / 2000.0))


def off():
    global _pwm, _cur
    if _pwm:
        _pwm.deinit()
        _pwm = None
        _cur = None
    print("서보 PWM 정지 (힘 빠짐)")
