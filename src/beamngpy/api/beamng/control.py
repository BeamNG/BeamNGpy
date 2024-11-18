from __future__ import annotations

from typing import Dict

from beamngpy.types import StrDict

from .base import Api


class ControlApi(Api):
    """
    An API allowing control of the flow of the simulation - pausing/resuming, stepping,
    and also enabling support for calling custom Lua code.

    Args:
        beamng: An instance of the simulator.
    """

    def step(self, count: int, wait: bool = True) -> None:
        """
        Advances the simulation the given amount of steps, assuming it is
        currently paused. If the wait flag is set, this method blocks until
        the simulator has finished simulating the desired amount of steps. If
        not, this method resumes immediately. This can be used to queue
        commands that should be executed right after the steps have been
        simulated.

        Args:
            count: The amount of steps to simulate.
            wait: Optional. Whether to wait for the steps to be
                  simulated. Defaults to True.

        Raises:
            BNGError: If the wait flag is set but the simulator doesn't respond
                      appropriately.
        """
        data: StrDict = dict(type="Step", count=count)
        data["ack"] = wait
        resp = self._send(data)
        if wait:
            resp.ack("Stepped")
        self._logger.info(f"Advancing the simulation by {count} steps.")

    def pause(self) -> None:
        """
        Sends a pause request to BeamNG.*, blocking until the simulation is
        paused.
        """
        data = dict(type="Pause")
        self._send(data).ack("Paused")
        self._logger.info("Pausing the simulation.")

    def resume(self) -> None:
        """
        Sends a resume request to BeamNG.*, blocking until the simulation
        is resumed.
        """
        data = dict(type="Resume")
        self._send(data).ack("Resumed")
        self._logger.info("Resuming the simulation.")

    def get_gamestate(self) -> Dict[str, str]:
        """
        Retrieves the current game state of the simulator. The game state is
        returned as a dictionary containing a ``state`` entry that is either:

            * ``scenario`` when a scenario is loaded
            * ``menu`` otherwise

        If a scenario is loaded, the resulting dictionary also contains a
        ``scenario_state`` entry whose value is ``pre-running`` if the scenario
        is currently at the start screen or ``running`` otherwise.

        Returns:
            The game state as a dictionary as described above.
        """
        data = dict(type="GameStateRequest")
        resp = self._send(data).recv("GameState")
        return resp

    def queue_lua_command(self, chunk: str, response: bool = False) -> StrDict:
        """
        Executes one lua chunk in the game engine VM.

        Args:
            chunk: lua chunk as a string
            response: If True, then the response is sent back to BeamNGpy.
        """
        data = dict(type="QueueLuaCommandGE")
        data["chunk"] = chunk
        data["resp"] = response
        return self._send(data).recv("ExecutedLuaChunkGE").get("resp", None)

    def return_to_main_menu(self) -> None:
        """
        Returns to the main menu, possibly closing the loaded scenario.
        """
        data = dict(type="StopScenario")
        self._send(data).ack("ScenarioStopped")

    def quit_beamng(self) -> None:
        """
        Sends the quit request to the simulator, which also closes the process.
        """
        data = dict(type="Quit")
        self._send(data).ack("Quit")
