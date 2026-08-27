# BMP388 기압계 드라이버 (I2C). read() -> (pressure_Pa, temperature_C)
# 보정식은 Bosch 공식 float 알고리즘. t_lin 이 곧 섭씨온도.
from time import sleep_ms
import config


def _u16(lo, hi):
    return lo | (hi << 8)


def _s8(v):
    return v - 256 if v & 0x80 else v


def _s16(lo, hi):
    v = lo | (hi << 8)
    return v - 65536 if v & 0x8000 else v


class BMP388:
    ADDRESS_LIST = (0x76, 0x77)
    CHIP_ID = 0x50

    def __init__(self, i2c):
        self.i2c = i2c
        self.address = self._find_address()
        self._write(0x7E, 0xB6)          # soft reset
        sleep_ms(10)
        self._read_calibration()
        self._write(0x1C, config.BMP_OSR)
        self._write(0x1D, config.BMP_ODR)
        self._write(0x1F, config.BMP_IIR)
        self._write(0x1B, 0x33)          # PWR_CTRL: press+temp en, normal mode
        sleep_ms(20)

        # ★ 설정이 실제로 먹었는지 확인한다.
        #   OSR/ODR 조합이 변환시간을 넘으면 conf_err 가 서고 데이터가
        #   0x800000(리셋값)으로 고정되는데, 그러면 압력이 그럴듯한 숫자로
        #   나와서 눈치채기 어렵다. 여기서 잡아야 한다.
        err = self._read(0x02, 1)[0]
        if err & 0x01:
            raise OSError("BMP388 fatal_err")
        if err & 0x04:
            raise OSError("BMP388 conf_err: OSR/ODR 변환시간 초과 (config.BMP_OSR/BMP_ODR)")
        pwr = self._read(0x1B, 1)[0]
        if (pwr & 0x30) != 0x30:
            raise OSError("BMP388 normal 모드 진입 실패 (PWR_CTRL=0x%02x)" % pwr)

        # 첫 변환 완료(STATUS bit5 = drdy_press)까지 대기
        for _ in range(30):
            if self._read(0x03, 1)[0] & 0x20:
                break
            sleep_ms(10)

    def _find_address(self):
        for a in self.ADDRESS_LIST:
            try:
                if self.i2c.readfrom_mem(a, 0x00, 1)[0] == self.CHIP_ID:
                    return a
            except OSError:
                pass
        raise OSError("BMP388 not found")

    def _read(self, register, length):
        return self.i2c.readfrom_mem(self.address, register, length)

    def _write(self, register, value):
        self.i2c.writeto_mem(self.address, register, bytes((value,)))

    def _read_calibration(self):
        d = self._read(0x31, 21)
        t1 = _u16(d[0], d[1]); t2 = _u16(d[2], d[3]); t3 = _s8(d[4])
        p1 = _s16(d[5], d[6]); p2 = _s16(d[7], d[8]); p3 = _s8(d[9])
        p4 = _s8(d[10]); p5 = _u16(d[11], d[12]); p6 = _u16(d[13], d[14])
        p7 = _s8(d[15]); p8 = _s8(d[16]); p9 = _s16(d[17], d[18])
        p10 = _s8(d[19]); p11 = _s8(d[20])
        self.t1 = t1 * 256.0
        self.t2 = t2 / 1073741824.0
        self.t3 = t3 / 281474976710656.0
        self.p1 = (p1 - 16384) / 1048576.0
        self.p2 = (p2 - 16384) / 536870912.0
        self.p3 = p3 / 4294967296.0
        self.p4 = p4 / 137438953472.0
        self.p5 = p5 * 8.0
        self.p6 = p6 / 64.0
        self.p7 = p7 / 256.0
        self.p8 = p8 / 32768.0
        self.p9 = p9 / 281474976710656.0
        self.p10 = p10 / 281474976710656.0
        self.p11 = p11 / 36893488147419103232.0

    def read(self):
        d = self._read(0x04, 6)
        raw_p = d[0] | (d[1] << 8) | (d[2] << 16)
        raw_t = d[3] | (d[4] << 8) | (d[5] << 16)

        # --- 온도 (t_lin = degC) ---
        x1 = raw_t - self.t1
        x2 = x1 * self.t2
        t_lin = x2 + x1 * x1 * self.t3

        # --- 압력 (Pa) ---
        x1 = self.p6 * t_lin
        x2 = self.p7 * t_lin * t_lin
        x3 = self.p8 * t_lin * t_lin * t_lin
        out1 = self.p5 + x1 + x2 + x3
        x1 = self.p2 * t_lin
        x2 = self.p3 * t_lin * t_lin
        x3 = self.p4 * t_lin * t_lin * t_lin
        out2 = raw_p * (self.p1 + x1 + x2 + x3)
        p2r = raw_p * raw_p
        x2 = self.p9 + self.p10 * t_lin
        x3 = p2r * x2
        x4 = x3 + raw_p * p2r * self.p11
        pressure = out1 + out2 + x4
        return pressure, t_lin

    def read_pressure(self):
        return self.read()[0]

    def data_ready(self):
        # STATUS(0x03): bit5=drdy_press, bit6=drdy_temp.
        # 데이터 레지스터를 읽으면 하드웨어가 자동으로 클리어한다.
        st = self._read(0x03, 1)[0]
        return (st & 0x20) != 0 and (st & 0x40) != 0

    def read_if_ready(self):
        # ★ 새 변환이 끝났을 때만 (pressure, temp) 반환, 아니면 None.
        #   센서 ODR(50Hz)과 메인루프(50Hz)는 동기화돼 있지 않아서 그냥 매 루프
        #   읽으면 같은 샘플을 두 번 이상 처리하게 된다. 그러면
        #     - median5 / 이동평균10 윈도우가 중복값으로 채워지고
        #     - BARO_CONFIRM(하강 5회 연속)이 실제 5개 샘플보다 적게 채워져
        #       사출 확인이 설계보다 헐거워진다.
        #   그래서 '새 샘플일 때만' 상위 로직에 넘긴다.
        if not self.data_ready():
            return None
        return self.read()
