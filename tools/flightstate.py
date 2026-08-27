# 비행상태 영구저장 (SD, 핑퐁 2슬롯 + 체크섬).
#  목적: 비행 중 브라운아웃/워치독 리셋 시 경과시간·기준값·사출여부를 복원해
#        (a) 타이머 재시작 (b) 센서 재영점 (c) 이중사출 을 모두 방지.
#  한 줄 텍스트: "1,state,elapsed_ms,ref_p,g0,deployed,checksum"
#  두 파일에 번갈아 저장 -> 쓰는 도중 전원이 끊겨도 반대편 슬롯은 무사.
import os
import config


class FlightState:
    def __init__(self, enabled):
        self.enabled = enabled
        self.slot = 0
        self.paths = (config.STATE_FILE_A, config.STATE_FILE_B)

    def _checksum(self, payload):
        c = 0
        for ch in payload:
            c = (c + ord(ch)) & 0xFFFF
        return c

    def save(self, state, elapsed_ms, ref_p, g0, deployed):
        if not self.enabled:
            return
        payload = "1,%d,%d,%.2f,%.4f,%d" % (
            state, elapsed_ms, ref_p, g0, 1 if deployed else 0)
        path = self.paths[self.slot]
        self.slot ^= 1                       # 다음엔 반대 슬롯
        try:
            f = open(path, "w")
            f.write(payload + ",%d\n" % self._checksum(payload))
            f.close()
        except Exception:
            pass

    def _read(self, path):
        try:
            f = open(path, "r")
            line = f.read()
            f.close()
        except Exception:
            return None
        line = line.strip()
        i = line.rfind(",")
        if i < 0:
            return None
        payload = line[:i]
        try:
            if int(line[i + 1:]) != self._checksum(payload):
                return None                  # 체크섬 불일치 = 손상
        except Exception:
            return None
        p = payload.split(",")
        if len(p) != 6 or p[0] != "1":
            return None
        try:
            return {
                "state": int(p[1]),
                "elapsed_ms": int(p[2]),
                "ref_p": float(p[3]),
                "g0": float(p[4]),
                "deployed": p[5] == "1",
            }
        except Exception:
            return None

    def load(self):
        # 유효한 슬롯 중 경과시간이 가장 큰(=가장 최근) 것
        best = None
        for p in self.paths:
            r = self._read(p)
            if r and (best is None or r["elapsed_ms"] > best["elapsed_ms"]):
                best = r
        return best

    def clear(self):
        for p in self.paths:
            try:
                os.remove(p)
            except Exception:
                pass
