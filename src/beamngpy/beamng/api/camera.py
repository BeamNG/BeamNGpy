from __future__ import annotations

from typing import TYPE_CHECKING, Dict

from beamngpy.types import Float3, Int3, Quat, StrDict

from .base import Api

if TYPE_CHECKING:
    from beamngpy.vehicle import Vehicle


class CameraApi(Api):
    def set_free(self, pos: Float3, direction: Float3) -> None:
        """
        Sets the position and direction of the free camera. The free camera is
        one that does not follow any particular vehicle, but can instead be
        put at any spot and any position on the map.

        Args:
            pos: The position of the camera as a (x, y, z) triplet.
            direction: The directional vector of the camera as a (x, y, z) triplet.
        """
        data: StrDict = dict(type='SetFreeCamera')
        data['pos'] = pos
        data['dir'] = direction
        self.send(data).ack('FreeCameraSet')

    def set_relative(self, pos: Float3, rot_quat: Quat | None = None) -> None:
        """
        Switches the camera mode for the currently-entered vehicle to the
        'relative' mode in which the camera can be placed at an arbitrary point
        relative to the vehicle, moving along with it as it drives around.

        Args:
            pos: (x, y, z) tuple of the camera's position relative to
                            the vehicle.
            rot_quat: The camera's rotation but written as a quat.
        """
        data: StrDict = dict(type='SetRelativeCam')
        data['pos'] = pos
        if rot_quat:
            data['rot'] = rot_quat
        self.send(data).ack('RelativeCamSet')

    def set_player_mode(self, vid: str, mode: str, config: StrDict, custom_data: StrDict | None = None) -> None:
        """
        Sets the camera mode of the vehicle identified by the given vehicle ID.
        The mode is given as a string that identifies one of the valid modes
        offered by the simulator. These modes can be queried using the
        (:meth:`~BeamNGpy.get_player_camera_mode`) method.

        The camera can be further configured with some common parameters,
        but it is not guaranteed the camera mode will respect all of them.
        These parameters include:

            * rotation: The rotation of the camera as a triplet of Euler angles
            * fov: The field of view angle
            * offset: The (x, y, z) vector to offset the camera's position by
            * distance: The distance of the camera to the vehicle

        Since each camera mode is implemented as a custom Lua extension, it is
        not possible to automatically query the exact features of the mode.
        Further information can be found in the
        lua/ge/extensions/core/cameraModes files which contain the
        implementations of each camera mode.

        Args:
            vid: Vehicle ID of the vehice to change the mode of.
            mode: Camera mode to set.
            config: Dictionary of further properties to set in the mode.
            custom_data: Custom data used by the specific camera mode. Defaults to None.
        """
        data: StrDict = dict(type='SetPlayerCameraMode')
        data['vid'] = vid
        data['mode'] = mode
        data['config'] = config
        data['customData'] = custom_data
        self.send(data).ack('PlayerCameraModeSet')

    def get_player_modes(self, vid: str) -> StrDict:
        """
        Retrieves information about the camera modes configured for the vehicle
        identified by the given ID.

        Args:
            vid: Vehicle ID of the vehicle to get camera mode information of.

        Returns:
            A dictionary mapping camera mode names to configuration options.
        """
        data = dict(type='GetPlayerCameraMode')
        data['vid'] = vid
        resp = self.send(data).recv('PlayerCameraMode')
        return resp['cameraData']

    def get_annotations(self) -> Dict[str, Int3]:
        """
        Method to obtain the annotation configuration of the simulator.

        Returns:
            A mapping of object classes to lists containing the [R, G, B]
            values of the colors objects of that class are rendered with.
        """
        data = dict(type='GetAnnotations')
        resp = self.send(data).recv('Annotations')
        return {key: tuple(int(v) for v in value) for key, value in resp['annotations'].items()}

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
