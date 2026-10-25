"""Humble-style ready snapshots over native rospy transport queues.

The transport threads enqueue only. Every pass considers ready timers before
ready subscriptions, in creation order, and consumes at most one message from
each subscription in that pass. Readiness is sampled again after the snapshot
has been traversed. Default callbacks are mutually exclusive; the behavior
node's explicitly separate events group may execute alongside its tick.
"""
from collections import deque
from concurrent.futures import ThreadPoolExecutor, wait
import threading
import time

import rospy


class ExternalShutdownException(Exception):
    pass


class Executor:
    def __init__(self):
        self._nodes = []
        self._condition = threading.Condition(threading.RLock())
        self._ready = deque()
        self._shutdown = False

    def add_node(self, node):
        with self._condition:
            if node in self._nodes:
                return False
            if node.executor is not None and node.executor is not self:
                node.executor.remove_node(node)
            self._nodes.append(node)
            node.executor = self
            self._condition.notify_all()
            return True

    def remove_node(self, node):
        with self._condition:
            if node in self._nodes:
                self._nodes.remove(node)
            if node.executor is self:
                node.executor = None
            self._ready = deque(entity for entity in self._ready if entity.node is not node)
            self._condition.notify_all()

    def get_nodes(self):
        with self._condition:
            return list(self._nodes)

    def wake(self):
        with self._condition:
            self._condition.notify_all()

    def _next(self, timeout_sec):
        deadline = None if timeout_sec is None or timeout_sec < 0 else time.monotonic() + timeout_sec
        with self._condition:
            while not self._shutdown:
                if rospy.is_shutdown():
                    raise ExternalShutdownException()
                self._check_futures()
                while self._ready:
                    entity = self._ready.popleft()
                    if entity.node._destroyed or entity.node not in self._nodes:
                        continue
                    if not entity.is_ready():
                        continue
                    if entity.callback_group.beginning_execution(entity):
                        if entity.reserve():
                            return entity
                        entity.callback_group.ending_execution(entity)
                # A new wait-set snapshot. Do not drain one entity's queue ahead
                # of the remaining ready entities (the source round-robin rule).
                # 이 snapshot을 다 순회한 뒤 readiness를 다시 조사한다.
                ready_entities = []
                for node in self._nodes:
                    for entity in tuple(node.timers) + tuple(node.subscriptions):
                        if entity.callback_group.can_execute(entity) and entity.is_ready():
                            ready_entities.append(entity)
                if ready_entities:
                    self._ready.extend(ready_entities)
                    continue
                remaining_timeout = None if deadline is None else deadline - time.monotonic()
                if remaining_timeout is not None and remaining_timeout <= 0:
                    return None
                # A short wall wait only services shutdown/clock notification.
                # Readiness and timer deadlines always use the ROS clock.
                wait_seconds = .05 if remaining_timeout is None else min(.05, remaining_timeout)
                for node in self._nodes:
                    for timer in node.timers:
                        if timer.callback_group.can_execute(timer):
                            timer_remaining_ns = timer.time_until_next_call()
                            if timer_remaining_ns is not None and timer_remaining_ns > 0:
                                wait_seconds = min(wait_seconds, timer_remaining_ns * 1e-9)
                self._condition.wait(wait_seconds)
        return None

    def _execute(self, entity):
        try:
            arguments = entity.take()
            if arguments is not None:
                entity.callback(*arguments)
        finally:
            entity.finish()
            entity.callback_group.ending_execution(entity)
            self.wake()

    def _check_futures(self):
        pass

    def spin(self):
        while not self._shutdown and not rospy.is_shutdown():
            self.spin_once()

    def shutdown(self, timeout_sec=None):
        with self._condition:
            self._shutdown = True
            self._condition.notify_all()
        return True


class SingleThreadedExecutor(Executor):
    def spin_once(self, timeout_sec=None):
        entity = self._next(timeout_sec)
        if entity is not None:
            self._execute(entity)


class MultiThreadedExecutor(Executor):
    def __init__(self, num_threads=2):
        super().__init__()
        self._pool = ThreadPoolExecutor(max_workers=num_threads)
        self._futures = []

    def spin_once(self, timeout_sec=None):
        self._check_futures()
        entity = self._next(timeout_sec)
        if entity is not None:
            self._futures.append(self._pool.submit(self._execute, entity))

    def _check_futures(self):
        # Application failures must reach main/roslaunch; never hide them in a
        # rospy callback thread or silently drop them in a Future.
        pending_futures = []
        for future in self._futures:
            if future.done():
                future.result()
            else:
                pending_futures.append(future)
        self._futures = pending_futures

    def shutdown(self, timeout_sec=None):
        super().shutdown(timeout_sec)
        _, pending = wait(self._futures, timeout=timeout_sec)
        self._pool.shutdown(wait=not pending)
        for future in self._futures:
            if future.done():
                future.result()
        return not pending
