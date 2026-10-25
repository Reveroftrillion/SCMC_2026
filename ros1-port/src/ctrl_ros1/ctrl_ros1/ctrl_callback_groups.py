"""Callback exclusion is applied by the executor, never by TCPROS threads."""
import threading


class MutuallyExclusiveCallbackGroup:
    def __init__(self):
        self._lock = threading.Lock()
        self._active_entity = None

    def can_execute(self, entity):
        with self._lock:
            return self._active_entity is None

    def beginning_execution(self, entity):
        with self._lock:
            if self._active_entity is not None:
                return False
            self._active_entity = entity
            return True

    def ending_execution(self, entity):
        with self._lock:
            if self._active_entity is not entity:
                raise RuntimeError('Callback group ownership mismatch')
            self._active_entity = None


class ReentrantCallbackGroup:
    def can_execute(self, entity):
        return True

    def beginning_execution(self, entity):
        return True

    def ending_execution(self, entity):
        pass
