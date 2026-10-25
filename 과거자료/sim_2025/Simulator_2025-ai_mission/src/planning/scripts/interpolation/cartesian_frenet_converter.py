"""

A converter between Cartesian and Frenet coordinate systems

author: Wang Zheng (@Aglargil)

Reference:

- [Optimal Trajectory Generation for Dynamic Street Scenarios in a Frenet Frame]
(https://www.researchgate.net/profile/Moritz_Werling/publication/224156269_Optimal_Trajectory_Generation_for_Dynamic_Street_Scenarios_in_a_Frenet_Frame/links/54f749df0cf210398e9277af.pdf)

"""

import math


class CartesianFrenetConverter:
    """
    A class for converting states between Cartesian and Frenet coordinate systems
    """

    @ staticmethod
    def cartesian_to_frenet(rs, rx, ry, rtheta, rkappa, rdkappa, x, y, v, a, theta, kappa):
        """
        Convert state from Cartesian coordinate to Frenet coordinate

        매개변수 (Parameters)
        ----------
        rs: 기준선(reference line)의 s-좌표 (경로를 따라가는 거리) -> 누적거리
        rx, ry: 기준점의 x, y 좌표
        rtheta: 기준점의 방향 (heading)
        rkappa: 기준점의 곡률 (curvature)
        rdkappa: 기준점의 곡률 변화율 (curvature rate)
        x, y: 현재 위치의 x, y 좌표
        v: 속도
        a: 가속도
        theta: 현재 방향 각도 (heading angle)
        kappa: 현재 곡률

        반환값 (Returns)
        -------
        s_condition: s에 대한 상태 [s(t), s'(t), s''(t)] (시간 t에 대한 거리, 속도, 가속도)
        d_condition: d에 대한 상태 [d(s), d'(s), d''(s)] (거리 s에 대한 측면 오프셋, 측면 속도, 측면 가속도)
        """
        dx = x - rx
        dy = y - ry

        cos_theta_r = math.cos(rtheta)
        sin_theta_r = math.sin(rtheta)

        cross_rd_nd = cos_theta_r * dy - sin_theta_r * dx
        d = math.copysign(math.hypot(dx, dy), cross_rd_nd)

        delta_theta = theta - rtheta
        tan_delta_theta = math.tan(delta_theta)
        cos_delta_theta = math.cos(delta_theta)

        one_minus_kappa_r_d = 1 - rkappa * d
        d_dot = one_minus_kappa_r_d * tan_delta_theta

        kappa_r_d_prime = rdkappa * d + rkappa * d_dot

        d_ddot = (-kappa_r_d_prime * tan_delta_theta +
                  one_minus_kappa_r_d / (cos_delta_theta * cos_delta_theta) *
                  (kappa * one_minus_kappa_r_d / cos_delta_theta - rkappa))

        s = rs
        s_dot = v * cos_delta_theta / one_minus_kappa_r_d

        delta_theta_prime = one_minus_kappa_r_d / cos_delta_theta * kappa - rkappa
        s_ddot = (a * cos_delta_theta -
                  s_dot * s_dot *
                  (d_dot * delta_theta_prime - kappa_r_d_prime)) / one_minus_kappa_r_d

        return [s, s_dot, s_ddot], [d, d_dot, d_ddot]

    @ staticmethod
    def frenet_to_cartesian(rs, rx, ry, rtheta, rkappa, rdkappa, s_condition, d_condition):
        """
        매개변수 (Parameters)
        ----------
        rs: 기준선(reference line)의 s-좌표 (경로를 따라가는 거리)
        rx, ry: 기준점의 x, y 좌표
        rtheta: 기준점의 방향 (heading)
        rkappa: 기준점의 곡률 (curvature)
        rdkappa: 기준점의 곡률 변화율 (curvature rate)
        s_condition: s에 대한 상태 [s(t), s'(t), s''(t)] (시간 t에 대한 거리, 속도, 가속도)
        d_condition: d에 대한 상태 [d(s), d'(s), d''(s)] (거리 s에 대한 측면 오프셋, 측면 속도, 측면 가속도)

        반환값 (Returns)
        -------
        x, y: 위치
        theta: 방향 각도 (heading angle)
        kappa: 곡률 (curvature)
        v: 속도 (velocity)
        a: 가속도 (acceleration)
        """
        if abs(rs - s_condition[0]) >= 1.0e-6:
            raise ValueError(
                "The reference point s and s_condition[0] don't match")

        cos_theta_r = math.cos(rtheta)
        sin_theta_r = math.sin(rtheta)

        x = rx - sin_theta_r * d_condition[0]
        y = ry + cos_theta_r * d_condition[0]

        one_minus_kappa_r_d = 1 - rkappa * d_condition[0]

        tan_delta_theta = d_condition[1] / one_minus_kappa_r_d
        delta_theta = math.atan2(d_condition[1], one_minus_kappa_r_d)
        cos_delta_theta = math.cos(delta_theta)

        theta = CartesianFrenetConverter.normalize_angle(delta_theta + rtheta)

        kappa_r_d_prime = rdkappa * d_condition[0] + rkappa * d_condition[1]

        kappa = (((d_condition[2] + kappa_r_d_prime * tan_delta_theta) *
                  cos_delta_theta * cos_delta_theta) / one_minus_kappa_r_d + rkappa) * \
            cos_delta_theta / one_minus_kappa_r_d

        d_dot = d_condition[1] * s_condition[1]
        v = math.sqrt(one_minus_kappa_r_d * one_minus_kappa_r_d *
                      s_condition[1] * s_condition[1] + d_dot * d_dot)

        delta_theta_prime = one_minus_kappa_r_d / cos_delta_theta * kappa - rkappa

        a = (s_condition[2] * one_minus_kappa_r_d / cos_delta_theta +
             s_condition[1] * s_condition[1] / cos_delta_theta *
             (d_condition[1] * delta_theta_prime - kappa_r_d_prime))

        return x, y, theta, kappa, v, a

    @ staticmethod
    def normalize_angle(angle):
        """
        Normalize angle to [-pi, pi]
        """
        a = math.fmod(angle + math.pi, 2.0 * math.pi)
        if a < 0.0:
            a += 2.0 * math.pi
        return a - math.pi
