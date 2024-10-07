from __future__ import annotations

from .base import VehicleApi


class LoggingApi(VehicleApi):
    """
    A base API class from which all the API communicating with a vehicle derive.

    Args:
        vehicle: An instance of a vehicle object.
    """

    def set_options_from_json(self, filename: str) -> None:
        """
        Updates the in game logging with the settings specified
        in the given file/json. The file is expected to be in
        the following location:
        <userpath>/<version_number>/<file_name>

        Args:
            filename
        """
        data = dict(type="ApplyVSLSettingsFromJSON", fileName=filename)
        self._send(data).ack("AppliedVSLSettings")

    def write_options_to_json(self, filename: str = "template.json") -> None:
        """
        Writes all available options from the in-game-logger to a json file.
        The purpose of this functionality is to facilitate the acquisition of
        a valid template to adjust the options/settings of the in game logging
        as needed.
        Depending on the executable used the file can be found at the following
        location:
        <userpath>/<BeamNG version number>/<fileName>

        Args:
            filename: not the absolute file path but the name of the json
        """
        data = dict(type="WriteVSLSettingsToJSON", fileName=filename)
        self._send(data).ack("WroteVSLSettingsToJSON")

    def start(self, output_dir: str) -> None:
        """
        Starts in game logging. Beware that any data
        from previous logging sessions is overwritten
        in the process.

        Args:
            output_dir: to avoid overwriting logging from other vehicles,
                        specify the output directory, overwrites the
                        output_dir set through the json. The data can be
                        found in: <userpath>/<BeamNG version number>/<output_dir>
        """
        data = dict(type="StartVSLLogging", outputDir=output_dir)
        self._send(data).ack("StartedVSLLogging")
        log_msg = (
            "Started in game logging."
            "The output for the vehicle stats logging can be found in "
            f"userfolder/<BeamNG version number>/{output_dir}."
        )
        self._logger.info(log_msg)

    def stop(self) -> None:
        """
        Stops in game logging.
        """
        data = dict(type="StopVSLLogging")
        self._send(data).ack("StoppedVSLLogging")
        self._logger.info("Stopped in game logging.")
