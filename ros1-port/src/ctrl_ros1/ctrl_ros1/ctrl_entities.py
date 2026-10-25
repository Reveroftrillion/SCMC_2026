"""Ingress queues and timers; application callbacks run only in an executor."""
from collections import deque
import threading

import rospy

from .ctrl_qos import DurabilityPolicy, profile


class Publisher:
    def __init__(self, msg_type, topic, qos):
        self.qos_profile = profile(qos)
        self._publisher = rospy.Publisher(
            topic, msg_type, queue_size=self.qos_profile.depth,
            latch=self.qos_profile.durability == DurabilityPolicy.TRANSIENT_LOCAL)

    def publish(self, msg):
        self._publisher.publish(msg)

    def get_subscription_count(self):
        return self._publisher.get_num_connections()

    def destroy(self):
        self._publisher.unregister()


class Subscription:
    def __init__(self, node, msg_type, topic, callback, qos, callback_group):
        self.node = node
        self.callback = callback
        self.callback_group = callback_group
        self.qos_profile = profile(qos)
        self._queue = deque(maxlen=self.qos_profile.depth)
        self._lock = threading.Lock()
        self._destroyed = False
        self._in_flight = False
        self._subscription = rospy.Subscriber(
            topic, msg_type, self.receive, queue_size=self.qos_profile.depth)

    def receive(self, msg):
        with self._lock:
            if self._destroyed:
                return
            self._queue.append(msg)
        self.node._wake()

    def is_ready(self):
        with self._lock:
            return bool(self._queue) and not self._destroyed and not self._in_flight

    def take(self):
        with self._lock:
            if self._destroyed or not self._queue:
                return None
            self._in_flight = True
            return (self._queue.popleft(),)

    def reserve(self):
        with self._lock:
            if self._destroyed or self._in_flight or not self._queue:
                return False
            self._in_flight = True
            return True

    def finish(self):
        with self._lock:
            self._in_flight = False

    def destroy(self):
        with self._lock:
            self._destroyed = True
            self._queue.clear()
        self._subscription.unregister()
        self.node._wake()


class Timer:
    def __init__(self, node, period_sec, callback, callback_group, clock):
        self.node = node
        self.callback = callback
        self.callback_group = callback_group
        self.clock = clock
        self.timer_period_ns = int(period_sec * 1_000_000_000)
        if self.timer_period_ns < 0:
            raise ValueError('Timer period must not be negative')
        self._lock = threading.Lock()
        self._canceled = False
        self._destroyed = False
        self._in_flight = False
        self._last_call = clock.now().nanoseconds
        self._next_call = self._last_call + self.timer_period_ns

    def on_clock_jump(self, old, new):
        # rcl_timer_time_jump: a backward jump before last_call schedules one
        # period from the new time. A forward jump leaves the deadline due.
        with self._lock:
            if new < old and new < self._last_call:
                self._last_call = new
                self._next_call = new + self.timer_period_ns

    def is_ready(self):
        now = self.clock.now().nanoseconds
        with self._lock:
            return (not self._destroyed and not self._canceled and
                    not self._in_flight and now >= self._next_call)

    def time_until_next_call(self):
        now = self.clock.now().nanoseconds
        with self._lock:
            if self._canceled or self._destroyed:
                return None
            return self._next_call - now

    def take(self):
        now = self.clock.now().nanoseconds
        with self._lock:
            if self._destroyed or self._canceled or now < self._next_call:
                return None
            self._in_flight = True
            self._last_call = now
            # Match rcl_timer_call, including equality at a missed deadline:
            # at next_call == now, one more callback remains immediately due.
            self._next_call += self.timer_period_ns
            if self._next_call < now:
                if self.timer_period_ns == 0:
                    self._next_call = now
                else:
                    # 이미 지난 deadline을 기존 정수 올림 계산으로 다음 주기에 맞춘다.
                    overdue_ns = now - self._next_call
                    self._next_call += ((overdue_ns + self.timer_period_ns - 1) //
                                        self.timer_period_ns) * self.timer_period_ns
            return ()

    def reserve(self):
        now = self.clock.now().nanoseconds
        with self._lock:
            if self._destroyed or self._canceled or self._in_flight or now < self._next_call:
                return False
            self._in_flight = True
            return True

    def finish(self):
        with self._lock:
            self._in_flight = False

    def cancel(self):
        with self._lock:
            self._canceled = True
        self.node._wake()

    def is_canceled(self):
        with self._lock:
            return self._canceled

    def reset(self):
        now = self.clock.now().nanoseconds
        with self._lock:
            self._next_call = now + self.timer_period_ns
            self._canceled = False
        self.node._wake()

    def destroy(self):
        with self._lock:
            self._destroyed = True
        self.node._wake()
