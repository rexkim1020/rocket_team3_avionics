# (2) 백업 사출 - 기압계 정점감지와 IMU가 모두 실패했을 때의 최후 수단.
#
#   ★ 두 갈래로 나눈다.
#   ㄱ) 하강 통과 백업: "BACKUP_ALT_M 을 넘은 적이 있고, 지금 그 아래"
#       -> '넘은 적 있음' 래치가 상승과 하강을 구분해 준다. 이게 없으면
#          정점이 BACKUP_ALT_M 근처인 비행에서 상승 중에 조건이 참이 되어
#          (예: 정점 119m 로켓은 T+4s에 109m 상승 14 m/s) 정상 트리거를
#          가로채고 낙하산을 상승 중에 터뜨린다.
#   ㄴ) 절대 시간 백스톱: 위 조건이 영영 성립하지 않는 경우
#       (연료 불량으로 정점이 BACKUP_ALT_M 에 못 미치는 비행)를 위해
#       BACKUP_HARD_MS 에 무조건 사출. 진짜 마지막 보루.
import config


class BackupDeploy:
    def __init__(self):
        self.reset()

    def reset(self):
        self.seen_above = False

    def update(self, flight_ms, altitude):
        # ㄴ) 절대 백스톱 - 고도를 못 읽어도 동작한다
        if flight_ms >= config.BACKUP_HARD_MS:
            return True
        if altitude is None:
            return False
        if altitude > config.BACKUP_ALT_M:
            self.seen_above = True
        # ㄱ) 하강 통과 백업
        return (self.seen_above
                and flight_ms >= config.BACKUP_TIME_MS
                and altitude <= config.BACKUP_ALT_M)
