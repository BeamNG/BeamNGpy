from __future__ import annotations

from typing import TYPE_CHECKING, Dict

from beamngpy.types import Float3, Int3, Quat, StrDict

from .base import Api

if TYPE_CHECKING:
    from beamngpy.vehicle import Vehicle


class CameraApi(Api):
    """
    An API class which allows control of the in-game camera and also provides
    information about the semantic annotation classes.

    Args:
        beamng: An instance of the simulator.
    """

    def set_free(self, pos: Float3, direction: Float3) -> None:
        """
        Sets the position and direction of the free camera. The free camera is
        one that does not follow any particular vehicle, but can instead be
        put at any spot and any position on the map.

        Args:
            pos: The position of the camera as a (x, y, z) triplet.
            direction: The directional vector of the camera as a (x, y, z) triplet.
        """
        data: StrDict = dict(type="SetFreeCamera")
        data["pos"] = pos
        data["dir"] = direction
        self._send(data).ack("FreeCameraSet")

    def set_relative(
        self, pos: Float3, dir: Float3, up: Float3 = (0.0, 0.0, 1.0)
    ) -> None:
        """
        Switches the camera mode for the currently-entered vehicle to the
        'relative' mode in which the camera can be placed at an arbitrary point
        relative to the vehicle, moving along with it as it drives around.

        Args:
            pos: (x, y, z) tuple of the camera's position relative to
                            the vehicle.
            dir (x, y, z): The cameras direction vector.
            up (x, y, z): The camera up vector (optional).
        """
        data: StrDict = dict(type="SetRelativeCam")
        data["pos"] = pos
        data["up"] = up
        if dir:
            data["dir"] = dir
        self._send(data).ack("RelativeCamSet")

    def set_player_mode(
        self,
        vehicle: str | Vehicle,
        mode: str,
        config: StrDict,
        custom_data: StrDict | None = None,
    ) -> None:
        """
        Sets the camera mode of the vehicle identified by the given vehicle ID.
        The mode is given as a string that identifies one of the valid modes
        offered by the simulator. These modes can be queried using the
        :meth:`.get_player_modes` method.

        The camera can be further configured with some common parameters,
        but it is not guaranteed the camera mode will respect all of them.
        These parameters include:

            * ``rotation``: The rotation of the camera as a triplet of Euler angles
            * ``fov``: The field of view angle
            * ``offset``: The (x, y, z) vector to offset the camera's position by
            * ``distance``: The distance of the camera to the vehicle

        Since each camera mode is implemented as a custom Lua extension, it is
        not possible to automatically query the exact features of the mode.
        Further information can be found in the
        ``lua/ge/extensions/core/cameraModes`` files which contain the
        implementations of each camera mode.

        Args:
            vehicle: Vehicle ID of the vehicle to change the mode of.
            mode: Camera mode to set.
            config: Dictionary of further properties to set in the mode.
            custom_data: Custom data used by the specific camera mode. Defaults to None.
        """
        data: StrDict = dict(type="SetPlayerCameraMode")
        data["vid"] = vehicle if isinstance(vehicle, str) else vehicle.vid
        data["mode"] = mode
        data["config"] = config
        data["customData"] = custom_data
        self._send(data).ack("PlayerCameraModeSet")

    def get_player_modes(self, vehicle: str | Vehicle) -> StrDict:
        """
        Retrieves information about the camera modes configured for the vehicle
        identified by the given ID.

        Args:
            vehicle: Vehicle ID of the vehicle to get camera mode information of.

        Returns:
            A dictionary mapping camera mode names to configuration options.
        """
        data = dict(type="GetPlayerCameraMode")
        data["vid"] = vehicle if isinstance(vehicle, str) else vehicle.vid
        resp = self._send(data).recv("PlayerCameraMode")
        return resp["cameraData"]

    def get_annotations(self) -> Dict[str, Int3]:
        """
        Method to obtain the annotation configuration of the simulator.

        Returns:
            A mapping of object classes to lists containing the ``[R, G, B]``
            values of the colors objects of that class are rendered with.
        """
        data = dict(type="GetAnnotations")
        resp = self._send(data).recv("Annotations")
        return {
            key: tuple(int(v) for v in value)
            for key, value in resp["annotations"].items()
        }

    def get_annotation_classes(self, annotations: Dict[str, Int3]) -> Dict[int, str]:
        """
        Method to convert the annotation configuration of the simulator into
        a mapping of colors to the corresponding object classes.

        Args:
            annotations: The annotation configuration of the simulator. Expected to be in the format
                         `get_annotations()` returns.

        Returns:
            A mapping of colors encoded as 24bit integers to object classes
            according to the simulator.
        """
        classes = {}
        for k, v in annotations.items():
            key = v[0] * 256 * 256 + v[1] * 256 + v[2]
            classes[key] = k
        return classes
