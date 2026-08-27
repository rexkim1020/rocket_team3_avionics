# 낙하산 사출 서보 (GP26, 직결/비반전 - 옵토 없음).
#
#   부팅 즉시 잠금(SAFE) 각으로 간다. 그래야 PWM 이 뜨기 전 Hi-Z 창에서
#   서보가 엉뚱하게 움직이지 않는다.
#
#   ★ 사출 후 복귀(relock)
#     사출각을 계속 유지하면 서보가 가동범위 끝에서 계속 힘을 쓴다.
#     낙하 중·착륙 후 내내 전류를 먹어 배터리를 갉아먹고, 같은 5V 레일을
#     쓰는 GPS·부저까지 영향을 받는다. 낙하산은 이미 빠져나갔으므로
#     SERVO_RELOCK_MS 뒤에 잠금 각으로 되돌린다(재포획 위험 없음).
#     0 으로 두면 복귀하지 않고 사출각을 유지한다.
from machine import Pin, PWM
from time import ticks_diff
import config


class ParachuteRelease:
    def __init__(self):
        self.done = False          # 사출 명령이 나갔는가(재사출 방지)
        self.deploy_ms = None      # 사출 시각
        self.relocked = False      # 복귀 완료
        self.servo = None
        if config.SERVO_ENABLED:
            self.servo = PWM(Pin(config.SERVO_PIN))
            self.servo.freq(config.SERVO_FREQ)
            self._us(config.SERVO_SAFE_US)

    def _us(self, pulse_us):
        if self.servo:
            self.servo.duty_ns(int(pulse_us) * 1000)

    def safe(self):
        self._us(config.SERVO_SAFE_US)

    def deploy(self, now_ms=None):
        if self.done:
            return
        self.done = True
        self.deploy_ms = now_ms
        self._us(config.SERVO_DEPLOY_US)

    def update(self, now_ms):
        # 매 루프 호출. 사출 후 정해진 시간이 지나면 잠금 각으로 복귀.
        if (config.SERVO_RELOCK_MS > 0 and self.done and not self.relocked
                and self.deploy_ms is not None
                and ticks_diff(now_ms, self.deploy_ms) >= config.SERVO_RELOCK_MS):
            self.relocked = True
            self._us(config.SERVO_SAFE_US)
            return True            # 복귀했음(로그용)
        return False

    def restore_deployed(self):
        # (미사용) 재부팅 복구용. 간소화판에는 복구 로직이 없다.
        self.done = True
        self._us(config.SERVO_DEPLOY_US)
