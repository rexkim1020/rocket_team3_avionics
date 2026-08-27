# (3) IMU 연직속도 기반 정점 판단 (BNO055 독립 계통).
#
#   판정 흐름 (팀 제안서 구조):
#     ① 상승 확인      v_peak >= IMU_MIN_SPEED
#     ② coast 감속 확인 a <= IMU_DECEL_MPS2 가 IMU_DECEL_HOLD_MS 지속
#     ③ 예상 정점 게이트 남은시간(v/|a|) <= EARLY_GUARD_MS 면 음수검사 개방
#     ④ 실제 하강 확인  v <= IMU_NEG_SPEED_MPS 가 IMU_NEG_HOLD_MS 지속 -> 사출
#
#   ★ 핵심 설계: 예상 정점은 '사출 판정'이 아니라 '음수속도 검사를 언제 열지'만
#     정한다. 예측은 틀릴 수 있으므로 예측만으로 쏘지 않고, 실제로 측정되는
#     사건(연직속도가 음수로 유지됨)을 최종 근거로 삼는다.
#
#   가속도는 LPF를 거치고 속도는 사다리꼴로 적분한다(사각형 적분의 편향 제거).
#   가속도계 포화가 관측되면 적분을 믿을 수 없으므로 스스로 무효화한다.
from time import ticks_diff
import config

_TWO_PI = 6.283185307179586


class BNODeployLogic:
    def __init__(self):
        self.reset(0, 0.0)

    def reset(self, now_ms, ground_accel):
        self.ground_accel = ground_accel   # 발사대 정지 시 연직 가속(중력분)
        self.last_ms = now_ms
        self.velocity = 0.0
        self.v_peak = 0.0
        self.a_filt = 0.0                  # LPF 통과한 연직 순가속
        self.a_prev = 0.0                  # 사다리꼴 적분용 직전값
        self._primed = False
        self.armed = False                 # ① 상승 확인됨
        self.decel_since = None
        self.coasting = False              # ② coast 확인됨
        self.gate_open = False             # ③ 음수속도 검사 개방됨
        self.neg_since = None              # ④ 음수속도 시작 시각
        self.t_remain = None               # 예상 잔여 상승시간(초) - 게이트/로그용
        self.saturated = False

    def update(self, now_ms, flight_ms, vert_accel):
        if abs(vert_accel) >= config.IMU_CLIP_MPS2:
            self.saturated = True          # 한 번이라도 포화하면 이후 계속 무효

        dt = ticks_diff(now_ms, self.last_ms) / 1000.0
        self.last_ms = now_ms
        if not (0.0 < dt < 0.2):
            return False

        a_raw = vert_accel - self.ground_accel     # 중력분 제거한 연직 순가속

        # --- 1차 저역통과 필터 ---
        if not self._primed:
            self.a_filt = a_raw
            self.a_prev = a_raw
            self._primed = True
        else:
            rc = 1.0 / (_TWO_PI * config.IMU_ACCEL_LPF_HZ)
            k = dt / (dt + rc)
            self.a_filt += k * (a_raw - self.a_filt)

        # --- 사다리꼴 적분 ---
        self.velocity += 0.5 * (self.a_prev + self.a_filt) * dt
        self.a_prev = self.a_filt
        if self.velocity > self.v_peak:
            self.v_peak = self.velocity

        if self.saturated:
            return False
        if flight_ms < config.IMU_ARM_AFTER_MS:
            return False                   # 발사 후 마진 시간 전에는 판정 안 함

        # ① 상승 확인 - 발사 전/직후 흔들림을 비행으로 착각하지 않기 위함
        if not self.armed:
            if self.v_peak < config.IMU_MIN_SPEED:
                return False
            self.armed = True

        # ② coast 감속 확인 - 짧은 가짜 감속을 배제하려고 지속시간을 본다
        if not self.coasting:
            if self.a_filt <= config.IMU_DECEL_MPS2:
                if self.decel_since is None:
                    self.decel_since = now_ms
                elif ticks_diff(now_ms, self.decel_since) >= config.IMU_DECEL_HOLD_MS:
                    self.coasting = True
            else:
                self.decel_since = None
            if not self.coasting:
                return False

        # ③ 예상 정점 게이트 (사출 아님, 검사 개방 시점만 결정)
        #    매 사이클 v/|a| 를 다시 계산한다. 정점에 가까울수록 항력이 사라져
        #    |a| -> g 로 수렴하므로 이 추정이 저절로 정확해진다.
        if not self.gate_open:
            if self.velocity <= 0.0:
                self.gate_open = True      # 이미 하강 중이면 바로 개방
            elif self.a_filt < 0.0:
                self.t_remain = self.velocity / (-self.a_filt)
                if self.t_remain <= (config.EARLY_GUARD_MS / 1000.0):
                    self.gate_open = True
            if not self.gate_open:
                return False

        # ④ 실제 하강 확인 - 순간적인 가짜 음수를 배제하려고 지속시간을 본다
        if self.velocity <= config.IMU_NEG_SPEED_MPS:
            if self.neg_since is None:
                self.neg_since = now_ms
            return ticks_diff(now_ms, self.neg_since) >= config.IMU_NEG_HOLD_MS
        self.neg_since = None
        return False
