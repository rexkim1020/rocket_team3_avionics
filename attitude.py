# 자이로 적분 자세추정 (쿼터니언).
#  발사 순간 reset() -> 그때의 자세를 기준(0도)으로 삼고, 이후 자이로만으로 적분.
#  ★ 단순 2축 적분(ang_x += wx*dt) 대신 쿼터니언을 쓰는 이유:
#    로켓은 축을 중심으로 스핀하는데, 스핀이 있으면 몸체좌표계 각속도를 그냥
#    더하는 방식은 롤 커플링 때문에 틀어진다. 쿼터니언은 이를 정확히 처리한다.
#  오차: 발사대에서 바이어스 영점을 잡으면 10초 비행에 대략 2~5도.
import math

_DEG2RAD = math.pi / 180.0


class Attitude:
    def __init__(self, axial_axis=2):
        self.axial = axial_axis
        self.reset()

    def reset(self):
        self.q = [1.0, 0.0, 0.0, 0.0]        # 현재 자세 = 기준 자세
        self.w_prev = (0.0, 0.0, 0.0)

    def update(self, wx, wy, wz, dt):
        # wx/wy/wz: 바이어스 보정된 몸체 각속도 [deg/s], dt: [s]
        if dt <= 0.0 or dt > 0.2:            # 비정상 dt(루프 스톨 등)는 버림
            self.w_prev = (wx, wy, wz)
            return
        # ★ 중점(midpoint) 각속도: 직전 샘플과 현재의 평균으로 회전량을 잡는다.
        #   50Hz에서 각속도가 빠르게 변할 때 사각형 적분보다 오차가 작다.
        pw = self.w_prev
        self.w_prev = (wx, wy, wz)
        x = 0.5 * (pw[0] + wx) * _DEG2RAD
        y = 0.5 * (pw[1] + wy) * _DEG2RAD
        z = 0.5 * (pw[2] + wz) * _DEG2RAD
        w0, x0, y0, z0 = self.q
        h = 0.5 * dt
        # q_dot = 0.5 * q (x) (0, wx, wy, wz)
        w = w0 + h * (-x0 * x - y0 * y - z0 * z)
        xn = x0 + h * (w0 * x + y0 * z - z0 * y)
        yn = y0 + h * (w0 * y - x0 * z + z0 * x)
        zn = z0 + h * (w0 * z + x0 * y - y0 * x)
        n = math.sqrt(w * w + xn * xn + yn * yn + zn * zn)
        if n <= 0.0:
            return
        self.q = [w / n, xn / n, yn / n, zn / n]

    def rotate(self, vx, vy, vz):
        # 몸체좌표 벡터를 기준(발사 순간) 좌표계로 회전: v' = q (x) v (x) q*
        # 이걸로 가속도의 '연직 성분'을 뽑아낸다. 로켓이 기울어도 유효.
        w, x, y, z = self.q
        tx = 2.0 * (y * vz - z * vy)
        ty = 2.0 * (z * vx - x * vz)
        tz = 2.0 * (x * vy - y * vx)
        return (vx + w * tx + (y * tz - z * ty),
                vy + w * ty + (z * tx - x * tz),
                vz + w * tz + (x * ty - y * tx))

    def tilt_deg(self):
        # 기준축과 현재 로켓 축 사이 각도. cos(tilt) = 1 - 2*(축과 무관한 두 성분의 제곱합)
        q = self.q
        if self.axial == 0:
            s = q[2] * q[2] + q[3] * q[3]
        elif self.axial == 1:
            s = q[1] * q[1] + q[3] * q[3]
        else:
            s = q[1] * q[1] + q[2] * q[2]
        c = 1.0 - 2.0 * s
        if c > 1.0:
            c = 1.0
        elif c < -1.0:
            c = -1.0
        return math.degrees(math.acos(c))
