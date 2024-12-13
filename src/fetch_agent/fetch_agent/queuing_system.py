import datetime
from typing import Tuple
import json


class QueuingSystem:
    reservations: list[Tuple[str, Tuple[int, int]]]

    def __init__(self, reservations: list[Tuple[str, Tuple[int, int]]]):
        self.reservations = reservations

    def open_time_frames(self):
        sorted_reservations: list[Tuple[str, Tuple[int, int]]] = sorted(
            self.reservations, key=lambda reservation_element: reservation_element[0]
        )

        open_time_frames = []
        beginning = int(datetime.datetime.now().timestamp())

        for reservation in sorted_reservations:
            reservation_timeframe: Tuple[int, int] = reservation[1]
            # check if reservation is starts before current beginning -> ignore skip it
            if reservation_timeframe[1] <= beginning:
                # if reservation end after beginning  (beginning is inbetween of a reservation)
                if reservation_timeframe[1] > beginning:
                    beginning = reservation_timeframe[1]

                continue

            open_time_frames.append((beginning, reservation[0]))
            beginning = reservation[1]

        open_time_frames.append(
            (
                beginning,
                int(
                    datetime.datetime.now()
                    .replace(year=datetime.datetime.now().year + 1)
                    .timestamp()
                ),
            )
        )

        return open_time_frames

    def does_conflict(self, start: int, duration: int) -> bool:
        if start < int(datetime.datetime.now().timestamp()):
            return True
        if start + duration > int(
            datetime.datetime.now()
            .replace(year=datetime.datetime.now().year + 1)
            .timestamp()
        ):
            return True
        for reservation in self.reservations:
            if not (
                start >= reservation[1][0] or start + duration <= reservation[1][0]
            ):
                return True
        return False

    def add_timeframe_for_sender(self, sender: str, time_frame: Tuple[int, int]):
        self.reservations.append((sender, time_frame))

    def remove_timeframe_for_sender(self, sender: str, time_frame: Tuple[int, int]):
        self.reservations.remove((sender, time_frame))

    def to_json(self):
        return json.dumps(self, default=lambda o: o.__dict__, sort_keys=True, indent=4)

    @staticmethod
    def from_json(data: str):
        return json.loads(data, object_hook=lambda d: QueuingSystem(**d))
