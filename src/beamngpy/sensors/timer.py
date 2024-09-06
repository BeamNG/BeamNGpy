from .sensor import Sensor


class Timer(Sensor):
    """
    The timer sensor keeps track of the time that has passed since the
    simulation started. It provides that information in seconds relative to the
    scenario start and does not represent something like a day time or date. It
    properly handles pausing the simulation, meaning the value of the timer
    sensor does not progress while the simulation is paused.

    When polled, this sensor provides the time in seconds since the start of
    the scenario in a dictionary under the ``time`` key.
    """

    def __init__(self):
        super().__init__()

    def encode_engine_request(self):
        req = dict(type="Timer")
        return req
