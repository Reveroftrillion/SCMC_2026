"""GPS 수신 + map 절대좌표(x, y) 변환을 담당하는 Gps 클래스."""

from pyproj import Transformer

import config
from lib.network.UDP import Receiver
from lib.define.GPS import GPS


class Gps:
    # WGS84 -> UTM 52N. 모든 인스턴스가 같은 변환을 쓰므로 클래스 속성으로 한 번만 생성.
    _transformer = Transformer.from_crs(config.GPS_SRC_CRS, config.GPS_DST_CRS, always_xy=True)

    def __init__(self, ip, port=None):
        self.ip = ip
        self.port = port or config.GPS_PORT
        self._receiver = Receiver(self.ip, self.port, GPS())

    def latlon_to_map(self, lat: float, lon: float):
        """WGS84 위경도를 map 원점 기준 (x, y)[m] 로 변환한다."""
        utm_x, utm_y = self._transformer.transform(lon, lat)
        map_x = utm_x - config.MAP_ORIGIN_X
        map_y = utm_y - config.MAP_ORIGIN_Y
        return map_x, map_y

    @staticmethod
    def _has_fix(lat, lon):
        # 초기화 직후(첫 패킷 수신 전) 구조체는 lat/lon 이 0.0 으로 남아있다.
        return not (lat == 0.0 and lon == 0.0)

    @staticmethod
    def safe_parse(gps_data):
        """
        gps_data.parsing() 을 안전하게 호출한다.

        lib/define/GPS.py 의 GPRMC.parsing() 은 체크섬이 'A*1B' 처럼 앞 필드에
        붙어 오는 표준 NMEA 문장에서 check_sum 필드 인덱스가 범위를 벗어나
        IndexError 를 던진다(lat/lon 은 그 전에 이미 채워짐). UDP 수신 프로세스
        안에서는 한번 삼켜지지만, 여기서 다시 호출하면 그대로 죽으므로 방어한다.
        """
        try:
            gps_data.parsing()
        except Exception:
            pass

    def get_position(self):
        """
        가장 최근 GPS 패킷에서 유효한 위치를 뽑아 (map_x, map_y, source) 를 반환한다.
        유효한 fix 가 없으면 None.

        - GPRMC 는 posStatus == 'A' 일 때만 유효
        - GPGGA 는 quality 가 '0'(없음) 이 아닐 때 유효
        - GPRMC 를 우선하고, 없으면 GPGGA 로 fallback
        """
        gps_data = self._receiver.get_data()
        self.safe_parse(gps_data)

        gprmc = gps_data.gprmc
        if gprmc.posStatus == 'A' and self._has_fix(gprmc.lat, gprmc.lon):
            x, y = self.latlon_to_map(gprmc.lat, gprmc.lon)
            return x, y, 'GPRMC'

        gpgga = gps_data.gpgga
        if gpgga.quality not in (None, '0', '') and self._has_fix(gpgga.lat, gpgga.lon):
            x, y = self.latlon_to_map(gpgga.lat, gpgga.lon)
            return x, y, 'GPGGA'

        return None
