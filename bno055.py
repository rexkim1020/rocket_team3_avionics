# BNO055 드라이버 - AMG(비융합) 모드.
#  ★ 융합 모드는 가속 레인지가 ±4g로 강제되어 로켓 연소 중(~6g, 피크 10g+)
#    포화된다. 그래서 AMG(0x07)로 두고 ±16g를 직접 설정한다.
#  ★ 대신 중력벡터/오일러각이 없으므로 자세(tilt)는 자이로 적분으로 구한다
#    -> attitude.py (발사 순간 수직 0도로 리셋 후 쿼터니언 적분)
#  단위: 가속도 m/s^2 (100 LSB = 1 m/s^2), 자이로 dps (16 LSB = 1 dps)
from time import sleep_ms
import struct
import math
import config


# 6면 정적 보정값(있으면 적용). imu_calibrate_6face.run() 이 생성한다.
#   참값 = (측정 - BIAS) / SCALE
try:
    import imu_cal
    _CAL_B = imu_cal.BIAS
    _CAL_S = imu_cal.SCALE
    _HAS_CAL = True
except Exception:
    _CAL_B = (0.0, 0.0, 0.0)
    _CAL_S = (1.0, 1.0, 1.0)
    _HAS_CAL = False


class BNO055:
    ADDRESS_LIST = (0x28, 0x29)
    CHIP_ID = 0xA0
    HAS_CAL = _HAS_CAL

    CONFIGMODE = 0x00
    MODE_AMG = 0x07          # accel + gyro + mag, 융합 없음

    def __init__(self, i2c, axial_axis=2):
        self.i2c = i2c
        self.axial = axial_axis
        sleep_ms(700)                        # 파워온 부팅 대기
        self.address = self._find_address()

        self._write(0x3D, self.CONFIGMODE)   # CONFIG 모드 (센서설정은 여기서만)
        sleep_ms(25)
        self._write(0x3F, 0x80)              # 외부 크리스털 사용(Adafruit 보드)
        sleep_ms(10)

        self._write(0x07, 0x01)              # --- PAGE 1 (센서 설정) ---
        self._write(0x08, config.BNO_ACC_CONFIG)   # ACC: ±16g
        self._write(0x0A, config.BNO_GYR_CONFIG)   # GYR: 2000dps
        self._write(0x07, 0x00)              # --- PAGE 0 ---

        self._write(0x3B, 0x00)              # UNIT_SEL: m/s^2, dps, degC
        self._write(0x3D, self.MODE_AMG)
        sleep_ms(50)                         # 모드 전환 19ms + 센서 기동 여유

        # ★ 모드 전환 직후 첫 몇 샘플이 전부 0으로 나온다(실기 확인됨).
        #   유효 데이터가 나올 때까지 버린다. 안 그러면 초기화 직후 읽는 쪽이
        #   0,0,0 을 받아 '센서 고장'으로 오판한다.
        for _ in range(25):
            try:
                a, _w = self.read_accel_gyro()
                if (abs(a[0]) + abs(a[1]) + abs(a[2])) > 0.5:
                    break
            except OSError:
                pass
            sleep_ms(20)

    def _find_address(self):
        for a in self.ADDRESS_LIST:
            try:
                if self.i2c.readfrom_mem(a, 0x00, 1)[0] == self.CHIP_ID:
                    return a
            except OSError:
                pass
        raise OSError("BNO055 not found")

    def _write(self, register, value):
        self.i2c.writeto_mem(self.address, register, bytes((value,)))

    def _read(self, register, length):
        return self.i2c.readfrom_mem(self.address, register, length)

    def read_accel_gyro(self):
        # 0x08~0x19 = accel(6) + mag(6) + gyro(6) 연속 -> 한 번의 I2C 버스트
        d = self._read(0x08, 18)
        ax, ay, az = struct.unpack_from("<hhh", d, 0)
        wx, wy, wz = struct.unpack_from("<hhh", d, 12)
        return ((( ax / 100.0 - _CAL_B[0]) / _CAL_S[0],
                 ( ay / 100.0 - _CAL_B[1]) / _CAL_S[1],
                 ( az / 100.0 - _CAL_B[2]) / _CAL_S[2]),
                (wx / 16.0, wy / 16.0, wz / 16.0))

    def read_accel(self):
        ax, ay, az = struct.unpack("<hhh", self._read(0x08, 6))
        return ((ax / 100.0 - _CAL_B[0]) / _CAL_S[0],
                (ay / 100.0 - _CAL_B[1]) / _CAL_S[1],
                (az / 100.0 - _CAL_B[2]) / _CAL_S[2])

    def read_accel_raw(self):
        # 보정을 적용하지 않은 원시 가속도 (6면 보정 절차 전용)
        ax, ay, az = struct.unpack("<hhh", self._read(0x08, 6))
        return ax / 100.0, ay / 100.0, az / 100.0

    def axial_accel(self, sign=1.0):
        return self.read_accel()[self.axial] * sign

    def is_alive(self, samples=6):
        # ★ '주소는 응답하는데 데이터가 0으로 고정'되는 고장을 잡는다.
        #   BNO055 는 파워온 타이밍이 나쁘면 초기화가 성공한 것처럼 보이면서
        #   계속 0 만 내보낸다. 그 상태로 날면 비행 내내 IMU 로그가 0이 되고
        #   사출 트리거(3)이 죽는다. 중력 크기가 그럴듯한지로 판별한다.
        for _ in range(samples):
            try:
                a = self.read_accel()
                m = math.sqrt(a[0] * a[0] + a[1] * a[1] + a[2] * a[2])
                if 5.0 < m < 15.0:
                    return True
            except OSError:
                pass
            sleep_ms(20)
        return False

    def tilt_from_accel(self, accel=None):
        # 정지 상태(발사대)에서만 유효: 중력방향과 로켓 축 사이 각. 수직=0도.
        a = accel if accel is not None else self.read_accel()
        mag = math.sqrt(a[0] * a[0] + a[1] * a[1] + a[2] * a[2])
        if mag < 1.0:
            return None
        c = a[self.axial] / mag
        if c > 1.0:
            c = 1.0
        elif c < -1.0:
            c = -1.0
        return math.degrees(math.acos(abs(c)))
