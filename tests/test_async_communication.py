from random import randbytes, randrange
from time import time

from beamngpy import BeamNGpy


def main():
    DATA_BYTES = 1024
    MESSAGES_SENT_BEFORE_RCVD = 100

    beamng = BeamNGpy('localhost', 64256)
    beamng.open()

    sent_bytes = 0
    rcvd_bytes = 0
    start_time = time()

    while True:
        sent_data = []
        for _ in range(MESSAGES_SENT_BEFORE_RCVD):
            data = randbytes(DATA_BYTES)
            delay = randrange(0, 120)
            request = {'type': 'Echo', 'data': data, 'delay': delay}
            response = beamng.connection.send(request)
            sent_data.append((response, data))
            sent_bytes += len(data)

        for (response, data) in sent_data:
            result = response.recv()
            rcvd_bytes += len(result['data'])
            assert result['data'] == data

        curr_time = time() - start_time
        speed = int(sent_bytes / curr_time)
        print(f'{sent_bytes:10} sent | {rcvd_bytes:10} rcvd | {speed:5} bps')


if __name__ == '__main__':
    main()
