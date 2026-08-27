# 부저 (GP20, 액티브 하이). 패턴 재생.
#
#   ★ 타이머로 분리해서 재생한다.
#     메인 루프는 SD 쓰기·GPS 파싱 때문에 주기가 25~32ms로 흔들린다.
#     루프에서 부저를 갱신하면 그 지터가 그대로 리듬 흔들림이 된다
#     (80ms 짜리 삑에 30ms 지터 = 37% 오차). 10ms 타이머로 돌리면
#     루프가 얼마나 바쁘든 리듬이 일정하다.
#
#   ★ 액티브 부저는 최대 음량까지 올라오는 데 시간이 걸린다.
#     짧은 삑(<100ms)은 소리가 작게 들린다. 중요한 알림은 길게 낸다.
#
#   패턴 = (on, off, on, off, ...) ms. 짝수 인덱스 = 소리 ON.
from machine import Pin, Timer
from time import ticks_ms, ticks_diff, sleep_ms
import config


class Buzzer:
    PATTERNS = {
        # --- 부팅 자가진단 결과 (3단계) ---
        #  ★ 결과음은 정해진 시간 동안 반복 재생되므로 '몇 번 울렸나'로는 구분할
        #    수 없다(6초 동안 2~3회 반복됨). 그래서 '리듬'으로 구분되게 만든다.
        #      sensorok = 하나가 뚝 떨어져서 울림   삐———  ...쉼...  삐———
        #      postok   = 둘이 붙어서 울림          삐—삐—  ...쉼...  삐—삐—
        "sensorok": (800, 2200),                    # 센서 정상, GPS 위성 대기중
        "postok":   (600, 250, 600, 2000),          # 전부 준비 완료 (발사 가능)
        "postfail": (130, 130, 130, 130, 130,
                     130, 130, 130, 130, 1200),     # 삐삐삐삐삐 (짧게 5번) 고장
        # --- 엄빌리컬 상태 ---
        "idle":     (60, 2400),                     # 발사불가(엄빌 미체결로 부팅)
        "s0":       (200, 400),                     # Stage0 대기
        "wait":     (100, 150),                     # Stage1 준비 안 됨(GPS 미고정)
        # Stage1 준비완료: 삐빅- 삐빅- 삐빅- ... 쉼
        "ready":    (120, 100, 120, 300,
                     120, 100, 120, 300,
                     120, 100, 120, 1000),
        "silent":   None,                           # Stage2 발사중
        "beacon":   (600, 600),                     # Stage3 착륙 비콘
    }

    def __init__(self):
        self.pin = Pin(config.BUZZER_PIN, Pin.OUT)
        self.pin.value(0)
        self.mode = None
        self._bounds = None      # 패턴 경계 누적합(미리 계산 -> 콜백에서 할당 없음)
        self._cycle = 0
        self.start = ticks_ms()
        self._manual = False     # 블로킹 재생 중에는 타이머가 손대지 않게
        self._timer = None

    # ---------- 패턴 재생 ----------
    def set(self, mode):
        if mode == self.mode:
            return               # 같은 모드면 타이밍 유지(재트리거 금지)
        self.mode = mode
        p = self.PATTERNS.get(mode)
        if not p:
            self._bounds = None      # 먼저 끈다 -> 콜백이 끼어들어도 조기 return
            self._cycle = 0
        else:
            b = []
            t = 0
            for d in p:
                t += d
                b.append(t)
            # ★ 순서 중요: _cycle 을 먼저 넣는다.
            #   반대로 하면 타이머 콜백이 두 대입 사이에 끼어들었을 때
            #   _bounds 는 새 값인데 _cycle 이 0 이라 0으로 나누기가 난다.
            self._cycle = t
            self._bounds = tuple(b)
        self.start = ticks_ms()
        self.pin.value(0)

    def update(self, _t=None):
        # 타이머 콜백 겸 수동 호출용. 메모리 할당 없이 동작한다.
        # 콜백에서 예외가 나면 부저가 영영 멈출 수 있으므로 통째로 감싼다.
        try:
            if self._manual:
                return
            b = self._bounds
            c = self._cycle
            if not b or c <= 0:
                self.pin.value(0)
                return
            pos = ticks_diff(ticks_ms(), self.start) % c
            i = 0
            n = len(b)
            while i < n:
                if pos < b[i]:
                    self.pin.value(1 if (i & 1) == 0 else 0)
                    return
                i += 1
        except Exception:
            pass

    # ---------- 타이머 ----------
    def start_timer(self, period_ms=10):
        if self._timer is None:
            self._timer = Timer(-1)
            self._timer.init(period=period_ms, mode=Timer.PERIODIC,
                             callback=self.update)

    def stop_timer(self):
        if self._timer is not None:
            self._timer.deinit()
            self._timer = None
        self.pin.value(0)

    # ---------- 블로킹 재생 (부팅 시에만) ----------
    def _play(self, pattern):
        self._manual = True
        try:
            for i, dur in enumerate(pattern):
                self.pin.value(1 if (i & 1) == 0 else 0)
                sleep_ms(dur)
            self.pin.value(0)
        finally:
            self._manual = False
            self.mode = None     # 이후 set() 이 다시 트리거되도록

    def post_ok(self):
        # 전부 정상: 삐— 삐— (길고 크게 2번)
        self._play((700, 350, 700))

    def post_fail(self, repeats=2):
        # 하나라도 문제: 삐삐삐삐삐 (짧게 5번)
        for r in range(repeats):
            self._play((130, 130, 130, 130, 130, 130, 130, 130, 130))
            if r < repeats - 1:
                sleep_ms(700)

    def long(self, ms=800):
        self._play((ms,))

    def code(self, count, on=130, off=130):
        p = []
        for _ in range(count):
            p.append(on)
            p.append(off)
        self._play(tuple(p[:-1]))
