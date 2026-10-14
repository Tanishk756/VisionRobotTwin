"""Deterministic delivery ordering on a monotonic simulation clock."""
from dataclasses import dataclass
import heapq
import itertools

from robotics.research.contracts import nonnegative


@dataclass(frozen=True)
class Delivery:
    scheduled_time_s: float
    actual_time_s: float
    payload: object


class EventQueue:
    """FIFO ties; late frames may reorder when their scheduled times differ.

    A consumer supplies its clock. Events are processed at the first drain at
    or after their timestamp; actual and requested times remain distinct.
    """
    def __init__(self):
        self._events = []
        self._counter = itertools.count()
        self._last_drain = 0.0

    def schedule(self, delivery_time_s, payload):
        nonnegative("delivery_time_s", delivery_time_s)
        if delivery_time_s < self._last_drain:
            raise ValueError("Cannot schedule an event in the processed past")
        heapq.heappush(self._events, (delivery_time_s, next(self._counter), payload))

    def drain(self, now_s):
        nonnegative("now_s", now_s)
        if now_s < self._last_drain:
            raise ValueError("Simulation clock cannot move backwards")
        self._last_drain = now_s
        deliveries = []
        while self._events and self._events[0][0] <= now_s + 1e-12:
            scheduled, _, payload = heapq.heappop(self._events)
            deliveries.append(Delivery(scheduled, now_s, payload))
        return deliveries

    def __len__(self):
        return len(self._events)

    def clear(self):
        """Cancel pending events without changing the monotonic clock."""
        count = len(self._events)
        self._events.clear()
        return count
