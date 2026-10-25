"""Current traffic order for committed entry, independent of object identity."""
import math


def rear_traffic(event, ctx, circle=None):
    # Compare centers, not an inflated future uncertainty envelope. The caller
    # enables this policy only after admission; admission itself checks both cars.
    x = ctx.x + 1.4825 * math.cos(ctx.yaw)
    y = ctx.y + 1.4825 * math.sin(ctx.yaw)
    if circle is not None:
        ego = math.atan2(y-circle.center[1], x-circle.center[0])
        other = math.atan2(event.y-circle.center[1], event.x-circle.center[0])
        delta = (other-ego)*circle.direction
        return math.atan2(math.sin(delta), math.cos(delta)) < 0.
    return (event.x-x)*math.cos(ctx.yaw)+(event.y-y)*math.sin(ctx.yaw) < 0.
