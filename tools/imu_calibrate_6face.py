# 6면 정적 가속도 보정 - 센서/PCB 조립 후 1회만 수행.
#
#   원리: 정지한 센서에는 중력만 작용하므로, 각 축을 위/아래로 향하게 하면
#   그 축이 +g / -g 를 읽어야 한다. 실제 읽은 값에서 bias 와 scale 을 구한다.
#       측정 = scale * 참값 + bias
#       bias[i]  = (측정[+i위] + 측정[-i위]) / 2      <- 두 방향 평균은 0이어야 함
#       scale[i] = (측정[+i위] - 측정[-i위]) / (2g)   <- 차이는 2g여야 함
#   보정 적용:  참값 = (측정 - bias) / scale
#
#   ※ 축간오차(cross-axis)는 보정하지 않는다. MEMS 가속도계에서 축간오차는
#     보통 1% 미만으로 bias/scale 보다 훨씬 작아, 최소제곱 3x3 추정을 넣을
#     만한 이득이 없다.
#
#   사용법 (Thonny 또는 mpremote repl 에서):
#       import imu_calibrate_6face
#       imu_calibrate_6face.run()
#   끝나면 imu_cal.py 가 생성된다. 그 뒤 피코를 재부팅할 것.
from machine import Pin, I2C
from time import sleep_ms
import math
import config

G = 9.80665
FACES = (
    ("+X 축이 위(하늘)를 향하게", 0, +1),
    ("-X 축이 위를 향하게", 0, -1),
    ("+Y 축이 위를 향하게", 1, +1),
    ("-Y 축이 위를 향하게", 1, -1),
    ("+Z 축이 위를 향하게", 2, +1),
    ("-Z 축이 위를 향하게", 2, -1),
)


def _raw_sensor():
    # 보정을 적용하지 않은 원시값을 읽어야 하므로 드라이버의 raw 경로를 쓴다
    from bno055 import BNO055
    i2c = I2C(config.BNO_I2C_ID, sda=Pin(config.BNO_SDA_PIN),
              scl=Pin(config.BNO_SCL_PIN), freq=config.I2C_FREQ)
    return BNO055(i2c, config.IMU_AXIAL_AXIS)


def _measure(b, samples=150):
    s = [0.0, 0.0, 0.0]
    n = 0
    for _ in range(samples):
        try:
            a = b.read_accel_raw()
            for i in range(3):
                s[i] += a[i]
            n += 1
        except OSError:
            pass
        sleep_ms(20)
    if n == 0:
        raise OSError("가속도 읽기 실패")
    return [v / n for v in s]


def run():
    b = _raw_sensor()
    print("=== 6면 정적 가속도 보정 ===")
    print("각 자세에서 센서를 완전히 정지시키고 Enter 를 누르세요.")
    print("한 면이라도 움직이면 결과가 나빠집니다.\n")

    meas = {}
    for label, axis, sign in FACES:
        input("  [%s] 준비되면 Enter > " % label)
        a = _measure(b)
        meas[(axis, sign)] = a
        print("    측정: %+.3f %+.3f %+.3f   (|a|=%.3f)"
              % (a[0], a[1], a[2], math.sqrt(a[0]**2 + a[1]**2 + a[2]**2)))
        # 목표 축이 실제로 중력을 보고 있는지 확인
        if abs(a[axis]) < 7.0 or (a[axis] > 0) != (sign > 0):
            print("    ! 경고: 자세가 잘못된 것 같습니다 (목표축 값 %+.2f)" % a[axis])

    bias = [0.0, 0.0, 0.0]
    scale = [1.0, 1.0, 1.0]
    for i in range(3):
        up = meas[(i, +1)][i]
        dn = meas[(i, -1)][i]
        bias[i] = (up + dn) / 2.0
        scale[i] = (up - dn) / (2.0 * G)

    print("\n--- 결과 ---")
    print("bias  = %+.4f %+.4f %+.4f  m/s^2" % (bias[0], bias[1], bias[2]))
    print("scale = %.5f %.5f %.5f" % (scale[0], scale[1], scale[2]))

    # 보정 후 잔차 확인
    worst = 0.0
    for (axis, sign), a in meas.items():
        c = [(a[i] - bias[i]) / scale[i] for i in range(3)]
        mag = math.sqrt(c[0]**2 + c[1]**2 + c[2]**2)
        err = abs(mag - G)
        if err > worst:
            worst = err
    print("보정 후 |a| 최대오차 = %.4f m/s^2 (0.05 이하면 양호)" % worst)
    if worst > 0.15:
        print("! 잔차가 큽니다. 자세를 더 정확히 잡고 다시 측정하는 것을 권합니다.")

    f = open("imu_cal.py", "w")
    f.write("# imu_calibrate_6face.run() 이 자동 생성함. 직접 수정하지 말 것.\n")
    f.write("BIAS = (%.6f, %.6f, %.6f)\n" % (bias[0], bias[1], bias[2]))
    f.write("SCALE = (%.6f, %.6f, %.6f)\n" % (scale[0], scale[1], scale[2]))
    f.write("RESIDUAL = %.6f\n" % worst)
    f.close()
    print("\nimu_cal.py 저장 완료. 피코를 재부팅하세요.")
