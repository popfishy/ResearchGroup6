#!/usr/bin/python3

"""
Convert GPS position information to XYZ position information, and back.
Author: buaa_szx
time:   2022.1.25
Modified from the following code:
https://github.com/PX4/PX4-Autopilot/blob/master/src/lib/geo/geo.cpp
"""

import math
import numpy as np


class PositionConvert:
    CONSTANTS_RADIUS_OF_EARTH = 6371000.0  # 地球半径，单位：米

    def __init__(self, ref_lat, ref_lon, ref_alt):
        """
        初始化参考点的WGS84坐标（纬度、经度、高度）。
        """
        self.ref_lat = ref_lat
        self.ref_lon = ref_lon
        self.ref_alt = ref_alt

    def WGS84toXYZ(self, lat, lon, alt):
        """
        将WGS84坐标转换为XYZ坐标。

        参数:
        lat (float): 纬度，单位：度
        lon (float): 经度，单位：度
        alt (float): 高度，单位：米

        返回:
        tuple: (x, y, z) 坐标，单位：米  x:北 y:东 z:天
        """
        lat_rad = math.radians(lat)
        lon_rad = math.radians(lon)
        ref_lat_rad = math.radians(self.ref_lat)
        ref_lon_rad = math.radians(self.ref_lon)

        sin_lat = math.sin(lat_rad)
        cos_lat = math.cos(lat_rad)
        ref_sin_lat = math.sin(ref_lat_rad)
        ref_cos_lat = math.cos(ref_lat_rad)

        cos_d_lon = math.cos(lon_rad - ref_lon_rad)

        arg = np.clip(ref_sin_lat * sin_lat + ref_cos_lat * cos_lat * cos_d_lon, -1.0, 1.0)
        c = math.acos(arg) if abs(arg) <= 1 else 0  # 防止超出范围

        k = c / math.sin(c) if abs(c) > 0 else 1.0

        x = float(k * (ref_cos_lat * sin_lat - ref_sin_lat * cos_lat * cos_d_lon) * self.CONSTANTS_RADIUS_OF_EARTH)
        y = float(k * cos_lat * math.sin(lon_rad - ref_lon_rad) * self.CONSTANTS_RADIUS_OF_EARTH)
        z = alt - self.ref_alt

        return x, y, z

    def XYZtoWGS84(self, x, y, z):
        """
        将XYZ坐标转换为GPS坐标。

        参数:
        x (float): X坐标，单位：米
        y (float): Y坐标，单位：米
        ref_lat (float): 参考点纬度，单位：度
        ref_lon (float): 参考点经度，单位：度

        返回:
        tuple: (lat, lon) 坐标，单位：度
        """
        x_rad = float(x) / self.CONSTANTS_RADIUS_OF_EARTH
        y_rad = float(y) / self.CONSTANTS_RADIUS_OF_EARTH
        c = math.sqrt(x_rad * x_rad + y_rad * y_rad)

        ref_lat_rad = math.radians(self.ref_lat)
        ref_lon_rad = math.radians(self.ref_lon)

        ref_sin_lat = math.sin(ref_lat_rad)
        ref_cos_lat = math.cos(ref_lat_rad)

        if abs(c) > 0:
            sin_c = math.sin(c)
            cos_c = math.cos(c)

            lat_rad = math.asin(cos_c * ref_sin_lat + (x_rad * sin_c * ref_cos_lat) / c)
            lon_rad = ref_lon_rad + math.atan2(y_rad * sin_c, c * ref_cos_lat * cos_c - x_rad * ref_sin_lat * sin_c)

            lat = math.degrees(lat_rad)
            lon = math.degrees(lon_rad)

        else:
            lat = math.degrees(self.ref_lat)
            lon = math.degrees(self.ref_lon)
        alt = z + self.ref_alt

        return lat, lon, alt

    def NEDtoWGS84(self, n, e, d):
        x = n
        y = e
        z = -d
        return self.XYZtoWGS84(x, y, z)

    def ENUtoWGS84(self, e, n, u):
        x = n
        y = e
        z = u
        return self.XYZtoWGS84(x, y, z)
    
    def WGS84toENU(self, lat, lon, alt):
        """
        将WGS84坐标转换为ENU（East-North-Up）坐标系。

        参数:
        lat (float): 纬度，单位：度
        lon (float): 经度，单位：度
        alt (float): 高度，单位：米

        返回:
        tuple: (e, n, u) 坐标，单位：米
        """
        x, y, z = self.WGS84toXYZ(lat, lon, alt)
        e = y
        n = x
        u = z
        return e, n, u

    def WGS84toNED(self, lat, lon, alt):
        """
        将WGS84坐标转换为NED（North-East-Down）坐标系。

        参数:
        lat (float): 纬度，单位：度
        lon (float): 经度，单位：度
        alt (float): 高度，单位：米

        返回:
        tuple: (n, e, d) 坐标，单位：米
        """
        x, y, z = self.WGS84toXYZ(lat, lon, alt)
        n = x
        e = y
        d = -z
        return n, e, d

    def WGS84toCBF(self, lat, lon, alt, heading):
        """
        将WGS84坐标转换为CBF（Car Body Frame）坐标系。

        参数:
        lat (float): 纬度，单位：度
        lon (float): 经度，单位：度
        alt (float): 高度，单位：米
        heading (float): 车体朝向角，单位：度

        返回:
        tuple: (x_cbf, y_cbf, z_cbf) 坐标，单位：米
        """
        x, y, z = self.WGS84toXYZ(lat, lon, alt)

        heading_rad = math.radians(heading)
        cos_heading = math.cos(heading_rad)
        sin_heading = math.sin(heading_rad)

        x_cbf = x * cos_heading + y * sin_heading
        y_cbf = -x * sin_heading + y * cos_heading
        z_cbf = z

        return x_cbf, y_cbf, z_cbf
