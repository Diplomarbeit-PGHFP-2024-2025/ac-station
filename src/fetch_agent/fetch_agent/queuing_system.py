import datetime
from typing import Tuple
import json


class QueuingSystem:
    reservations: list[Tuple[int, int]]

    def __init__(self, reservations: list[Tuple[int, int]]):
        self.reservations = reservations

    def open_time_frames(self):
        sorted_reservations = sorted(self.reservations, key=lambda reservation_element: reservation_element[0])

        # fails if now is in reservation
        open_time_frames = []
        beginning = int(datetime.datetime.now().timestamp())
        for reservation in sorted_reservations:
            open_time_frames.append((beginning, reservation[0]))
            beginning = reservation[1]
        open_time_frames.append((beginning, int(datetime.datetime.now().replace(year=datetime.datetime.now().year + 1).timestamp())))

        return open_time_frames

    def does_conflict(self, start: int, duration: int) -> bool:
        if start < int(datetime.datetime.now().timestamp()):
            return True
        if start + duration > int(datetime.datetime.now().replace(year=datetime.datetime.now().year + 1).timestamp()):
            return True
        for reservation in self.reservations:
            if not (start >= reservation[0] or start + duration <= reservation[0]):
                return True
        return False

    def to_json(self):
        return json.dumps(self, default=lambda o: o.__dict__, sort_keys=True, indent=4)

    @staticmethod
    def from_json(data: str):
        return json.loads(data, object_hook=lambda d: QueuingSystem(**d))
