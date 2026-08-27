# 기압고도 미분 -> 수직속도.
#  ★ 단순 차분(h[n]-h[n-1])/dt 는 기압 노이즈를 그대로 증폭한다.
#    슬라이딩 윈도우 최소제곱(least squares) 기울기를 쓰면 같은 코드량으로
#    노이즈가 크게 줄어든다. 윈도우 N=15, 50Hz -> 300ms (지연 약 150ms).
#
#  용도: BNO055 가속도 적분 속도와 교차검증.
#    - 두 값이 크게 어긋나면 적분이 드리프트했다는 뜻 -> IMU 사출 트리거 무효화
#    - SD 로그에 둘 다 남겨 비행 후 분석
#  주의: 고속에서는 동압(ram pressure) 때문에 기압 속도에도 오차가 생긴다.
#        그래서 비교 임계값은 넉넉하게 잡는다.
from time import ticks_diff
import config


class BaroVelocity:
    def __init__(self, window=None):
        self.n = window if window else config.BARO_VEL_WINDOW
        self.reset()

    def reset(self):
        self.t = []
        self.h = []
        self.v = None

    def update(self, t_ms, altitude):
        if altitude is None:
            return self.v
        self.t.append(t_ms)
        self.h.append(altitude)
        if len(self.t) > self.n:
            self.t.pop(0)
            self.h.pop(0)
        if len(self.t) < self.n:
            return None

        t0 = self.t[0]
        n = len(self.t)
        st = 0.0
        sh = 0.0
        stt = 0.0
        sth = 0.0
        for i in range(n):
            x = ticks_diff(self.t[i], t0) / 1000.0
            y = self.h[i]
            st += x
            sh += y
            stt += x * x
            sth += x * y
        den = n * stt - st * st
        if den <= 0.0:
            return self.v
        self.v = (n * sth - st * sh) / den      # m/s (+ = 상승)
        return self.v
