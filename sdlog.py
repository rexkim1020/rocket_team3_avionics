# MicroSD 로깅 (SPI1). CSV 한 줄씩 append + 주기적 커밋(close/reopen).
#  ★ 표준 드라이버 sdcard.py 가 같은 폴더에 있어야 함(micropython-lib 공식).
#  ★ SD는 FAT32 포맷만 마운트됨(exFAT 안 됨). 32GB↑는 SD Card Formatter 사용.
#  SD가 없거나 실패해도 ok=False로 두고 비행은 계속 진행(벤치 편의).
import os
from machine import Pin, SPI
from time import sleep_ms, ticks_ms, ticks_diff
import config


class SDLog:
    #  ax/ay/az = 가속도 m/s^2 (±16g), wx/wy/wz = 각속도 deg/s (바이어스 보정됨)
    #  tilt_deg = 발사 전엔 가속도 기준(정지), 비행 중엔 자이로 적분(발사시 0도)
    #  vel_imu  = BNO055 축가속 적분 속도 / vel_baro = BMP388 고도 미분 속도
    HEADER = (
        "t_ms,stage,flight_ms,alt_m,pressure_pa,temp_c,"
        "ax,ay,az,wx,wy,wz,vert_a,vel_imu,vel_baro,tilt_deg,"
        "gps_fix,gps_sats,gps_lat,gps_lon,gps_alt,gps_utc,deploy\n"
    )

    def __init__(self):
        self.ok = False
        self.f = None
        self.last_flush = ticks_ms()
        # 커밋 주기. 발사 전에는 main 이 이 값을 크게 잡아 SPI 활동을 줄인다
        # (SD 노이즈가 GPS 위성 획득을 방해하므로).
        self.flush_ms = config.SD_FLUSH_MS
        try:
            import sdcard
            spi = SPI(
                config.SD_SPI_ID,
                baudrate=1_000_000,
                sck=Pin(config.SD_SCK_PIN),
                mosi=Pin(config.SD_MOSI_PIN),
                miso=Pin(config.SD_MISO_PIN),
            )
            sleep_ms(250)                    # 마운트 안정화
            sd = sdcard.SDCard(spi, Pin(config.SD_CS_PIN))
            os.mount(sd, config.SD_MOUNT)
            try:
                os.stat(config.SD_LOGFILE)
                is_new = False
            except OSError:
                is_new = True
            self.f = open(config.SD_LOGFILE, "a")
            if is_new:
                self.f.write(self.HEADER)
            self.f.flush()
            self.ok = True
            print("[SD] 로그 시작:", config.SD_LOGFILE)
        except Exception as e:
            print("[SD] 비활성(로깅 없이 진행):", e)
            self.ok = False

    def write(self, row):
        if not self.ok:
            return
        try:
            self.f.write(row)
            now = ticks_ms()
            if ticks_diff(now, self.last_flush) >= self.flush_ms:
                self.f.flush()
                self.f.close()               # 전원 급차단 대비 강제 커밋
                self.f = open(config.SD_LOGFILE, "a")
                self.last_flush = now
        except Exception as e:
            print("[SD] write 실패:", e)
            self.ok = False

    def event(self, text):
        if not self.ok:
            return
        try:
            self.f.write("# " + text + "\n")
            self.f.flush()
        except Exception:
            self.ok = False
