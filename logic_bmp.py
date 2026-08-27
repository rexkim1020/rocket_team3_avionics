# (1) BMP388 기압고도 기반 정점(apogee) 감지 - 오차 최소화 파이프라인:
#   원시고도 -> 중앙값필터(스파이크 제거) -> 이동평균(평활)
#         -> 최고점 추적 -> 최고점 대비 DROP_M 이상 낮은 값이 CONFIRM회 연속 -> 사출
import config


class BMPDeployLogic:
    def __init__(self):
        self.reset()

    def reset(self):
        self._med = []
        self._win = []
        self.smooth = None
        self.peak = -1.0e9
        self.count = 0
        self.apogee_alt = None

    def _median(self, x):
        self._med.append(x)
        if len(self._med) > config.BARO_MEDIAN:
            self._med.pop(0)
        s = sorted(self._med)
        return s[len(s) // 2]

    def update(self, altitude):
        m = self._median(altitude)                 # 1) 스파이크 제거
        self._win.append(m)                         # 2) 이동평균
        if len(self._win) > config.BARO_WINDOW:
            self._win.pop(0)
        if len(self._win) < config.BARO_WINDOW:
            return False
        self.smooth = sum(self._win) / len(self._win)

        if self.smooth > self.peak:                 # 3) 최고점 갱신
            self.peak = self.smooth
            self.count = 0
        elif self.smooth <= self.peak - config.BARO_DROP_M:   # 4) 하강 확인
            self.count += 1
        else:
            self.count = 0

        if self.count >= config.BARO_CONFIRM:
            self.apogee_alt = self.peak
            return True
        return False
