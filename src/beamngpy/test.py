"""
.. module:: test
    :platform: Windows
    :synopsis: Module containing classes that offer testing functionality
               for BeamNGpy.
"""

RESULT_NOTHING = 0
RESULT_FAILURE = 1
RESULT_SUCCESS = 2


class TestOracle:

    def is_success(self, bng, scenario):
        raise NotImplementedError('Subclasses need to implement this!')

    def is_failure(self, bng, scenario):
        raise NotImplementedError('Subclasses need to implement this!')


class BeamNGTest:

    def __init__(self, bng, scenario, rate):
        self.bng = bng
        self.scenario = scenario
        self.rate = rate

        self.oracles = set()

    def add_oracle(self, oracle):
        self.oracles.add(oracle)

    def remove_oracle(self, oracle):
        self.oracles.remove()

    def test_success(self):
        for oracle in self.oracles:
            result, reason = oracle.is_success(self.bng, self.scenario)
            if result:
                return result, reason

        return False, None

    def test_failure(self):
        for oracle in self.oracles:
            result, reason = oracle.is_failure(self.bng, self.scenario)
            if result:
                return result, reason

        return False, None

    def update(self):
        self.bng.step(self.rate)
        self.bng.update(self.scenario)
        failure, reason = self.test_failure()
        if failure:
            return RESULT_FAILURE, reason

        success, reason = self.test_success()
        if success:
            return RESULT_SUCCESS, reason

        return RESULT_NOTHING, None

    def run(self):
        result, reason = RESULT_NOTHING, None
        while result == RESULT_NOTHING:
            result, reason = self.update()
        return result, reason
