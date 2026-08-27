# 엄빌리컬 3개(SAFE/POGO1/POGO2) 디바운스 입력.
#  연결=LOW. 분류: 0=전부연결, 1=SAFE만분리, 2=전부분리, None=과도/불명
from machine import Pin
from time import ticks_ms, ticks_diff
import config


class StageInputs:
    def __init__(self):
        self.safety = Pin(config.SAFETY_PIN, Pin.IN, Pin.PULL_UP)
        self.pogo1 = Pin(config.POGO1_PIN, Pin.IN, Pin.PULL_UP)
        self.pogo2 = Pin(config.POGO2_PIN, Pin.IN, Pin.PULL_UP)
        self.last_raw = None
        self.changed = ticks_ms()

    def _con(self, pin):
        return pin.value() == config.CONNECTED_LEVEL

    def raw_connected(self):
        # 로깅/부팅판정용 즉시값 (safe, pogo1, pogo2)
        return self._con(self.safety), self._con(self.pogo1), self._con(self.pogo2)

    def _classify(self):
        s, p1, p2 = self.raw_connected()
        if s and p1 and p2:
            return 0
        if (not s) and p1 and p2:
            return 1
        if (not s) and (not p1) and (not p2):
            return 2
        return None

    def read(self):
        now = ticks_ms()
        raw = self._classify()
        if raw != self.last_raw:
            self.last_raw = raw
            self.changed = now
            return None
        if raw is None:
            return None
        if ticks_diff(now, self.changed) >= config.DEBOUNCE_MS:
            return raw
        return None
