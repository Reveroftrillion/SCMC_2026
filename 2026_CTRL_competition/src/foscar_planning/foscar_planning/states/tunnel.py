"""Tunnel mission state; no fixed speed cap by default.

Localization continues dead reckoning. Route and obstacle control determine
speed; a positive v_max_kmh explicitly opts into an additional tunnel cap.
"""
import math

from ..state import BehaviorState, Context, Target


class Tunnel(BehaviorState):
    name = 'TUNNEL'
    priority = 40
    defaults = {
        's_from': 1875.,        # 진입 s (GPS 끊기기 6 m 전)
        's_to': 2000.,          # 이후 + GPS 복귀면 해제
        'v_max_kmh': 0.,        # 0 disables the optional tunnel cap
    }

    def in_zone(self, ctx: Context) -> bool:
        return self.s_from <= ctx.s <= self.s_to

    def check_entry(self, ctx: Context) -> bool:
        return ctx.loc_mode == 'DR' or self.in_zone(ctx)

    def check_exit(self, ctx: Context) -> bool:
        return ctx.loc_mode != 'DR' and not self.in_zone(ctx)

    def compute_target(self, ctx: Context) -> Target:
        return Target(state=self.name, v_max=self.v_max_kmh / 3.6 if self.v_max_kmh > 0 else math.inf)
