# GPS (UART1, BE-220) 논블로킹 NMEA 파서. GGA(고정/좌표/고도) + RMC(좌표) 파싱.
# update()를 매 루프 호출. 최신 값 속성: fix, sats, lat, lon, alt, utc
# sentences = 유효(체크섬 통과) 문장 누적 수 -> 부팅 POST에서 '모듈 응답' 판정용.
from machine import UART, Pin
import config


class GPS:
    def __init__(self):
        self.uart = UART(
            config.GPS_UART_ID,
            baudrate=config.GPS_BAUD,
            tx=Pin(config.GPS_TX_PIN),
            rx=Pin(config.GPS_RX_PIN),
        )
        self.buf = b""
        self.sentences = 0
        self.fix = 0
        self.sats = 0
        self.lat = None
        self.lon = None
        self.alt = None
        self.utc = ""

    def update(self):
        try:
            n = self.uart.any()
            if n:
                data = self.uart.read(n)
                if data:
                    self.buf += data
                    while b"\n" in self.buf:
                        line, self.buf = self.buf.split(b"\n", 1)
                        self._parse(line)
            if len(self.buf) > 512:        # 폭주 방지
                self.buf = self.buf[-256:]
        except Exception:
            pass

    def _parse(self, line):
        try:
            s = line.decode().strip()
        except Exception:
            return
        if not s.startswith("$") or "*" not in s:
            return
        body, _star, cks = s[1:].partition("*")
        c = 0
        for ch in body:
            c ^= ord(ch)
        try:
            if c != int(cks[:2], 16):      # 체크섬 검증
                return
        except Exception:
            return
        self.sentences += 1                # 유효 문장 수신 = 모듈 살아있음
        f = body.split(",")
        t = f[0][-3:]
        if t == "GGA":
            self._gga(f)
        elif t == "RMC":
            self._rmc(f)

    def _coord(self, val, hemi):
        if not val:
            return None
        dot = val.find(".")
        if dot < 3:
            return None
        deg = int(val[:dot - 2])
        minutes = float(val[dot - 2:])
        d = deg + minutes / 60.0
        if hemi in ("S", "W"):
            d = -d
        return d

    def _gga(self, f):
        try:
            self.utc = f[1]
            self.fix = int(f[6]) if f[6] else 0
            self.sats = int(f[7]) if f[7] else 0
            if self.fix > 0:
                self.lat = self._coord(f[2], f[3])
                self.lon = self._coord(f[4], f[5])
                self.alt = float(f[9]) if f[9] else None
        except Exception:
            pass

    def _rmc(self, f):
        try:
            if f[2] == "A":                # A=유효
                self.lat = self._coord(f[3], f[4])
                self.lon = self._coord(f[5], f[6])
        except Exception:
            pass
