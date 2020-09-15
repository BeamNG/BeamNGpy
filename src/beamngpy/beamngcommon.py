"""
.. module:: beamngcommon
    :platform: Windows
    :synopsis: Module containing common functions used across various BeamNGpy
               modules.

.. moduleauthor:: Marc Müller <mmueller@beamng.gmbh>
.. moduleauthor:: Pascale Maul <pmaul@beamng.gmbh>
"""

import logging as log
import json
import os
import socket
import numpy as np
import warnings

from functools import wraps

import msgpack


ENV = dict()
ENV['BNG_HOME'] = os.getenv('BNG_HOME')


class BNGError(Exception):
    """
    Generic BeamNG error
    """
    pass


class BNGValueError(ValueError):
    """
    Value error specific to BeamNGpy.
    """
    pass


def raise_rot_deprecation_warning():
    warnings.warn('\'rot\' is deprecated, use rot_mat instead',
                  DeprecationWarning)


def ack(ack_type):
    def ack_wrapper(fun):
        @wraps(fun)
        def ack_wrapped(*args, **kwargs):
            ret = fun(*args, **kwargs)
            resp = args[0].recv()
            if resp['type'] != ack_type:
                raise BNGError('Wrong ACK: {} != {}'.format(ack_type,
                                                            resp['type']))
            return ret

        return ack_wrapped
    return ack_wrapper


class Config(dict):
    """
    Configuration class which mainly wraps around a dictionary to offer access
    to keys as members. Instead of `spam["eggs"]`, it"s possible to simply go
    `spam.eggs`.
    """

    def __getstate__(self):
        return self.__dict__.items()

    def __setstate__(self, items):
        for key, val in items:
            self.__dict__[key] = val

    def __getattr__(self, key):
        return super().__getitem__(key)

    def __setattr__(self, key, val):
        return super().__setitem__(key, val)

    def load_values(self, dic):
        """
        Loads every key-value pair from the given dictionary into the config.
        """
        for key, val in dic.items():
            self[key] = val

    def load(self, cfg_file):
        """
        Loads every key-value pair in the given json file into the config.
        """
        with open(cfg_file) as in_file:
            cfg_str = in_file.read()
            cfg_json = json.loads(cfg_str)
        self.load_values(cfg_json)

    def save(self, cfg_file):
        """
        Saves every key-value pair of this config into the given json file.
        """
        with open(cfg_file, "w") as out_file:
            cfg_str = json.dumps(self, indent=4, sort_keys=True)
            out_file.write(cfg_str)


def get_default():
    """
    Creates and returns an instance of the `Config` class with the default
    value for each option.
    """
    default = Config()
    # default.load_values(DEFAULT_CONFIG)
    return default


def ensure_config(cfg_file):
    """
    Tests if the given cfg_file path points to a configuration file. If not, a
    default configuration will be written to that file. The file is then loaded
    into the `CFG` field.
    """
    if not os.path.exists(cfg_file):
        default = get_default()
        default.save(cfg_file)
        log.debug("Saved fresh default cfg to: %s", cfg_file)

    CFG.load(cfg_file)


CFG = get_default()


def send_msg(skt, data):
    """
    Encodes the given data via messagepack and sends the bytes over the given
    socket. Before the raw message bytes are sent, the amount of bytes the
    message is long is sent as a zero-padded 16-character string.

    Args:
        skt (:class:`socket`): The socket to write to
        data (dict): The data to encode and send
    """
    data = msgpack.packb(data, use_bin_type=True)
    length = '{:016}'.format(len(data))
    skt.send(bytes(length, 'ascii'))
    skt.send(data)


def recv_msg(skt):
    """
    Reads a messagepack-encoded message from the given socket, decodes it, and
    returns it. Before the raw message bytes are read, this function expects
    the amount of bytes to read being sent as a zero-padded 16-character
    string.

    Args:
        skt (:class:`socket`): The socket to read from

    Returns:
        The decoded message.
    """
    length = skt.recv(16)
    length = int(str(length, 'ascii'))
    buf = bytearray()
    while length > 0:
        chunk = min(4096, length)
        received = skt.recv(chunk)
        buf.extend(received)
        length -= len(received)
    assert length == 0
    data = skt.recv(length)
    data = msgpack.unpackb(buf, raw=False)
    if 'bngError' in data:
        raise BNGError(data['bngError'])
    if 'bngValueError' in data:
        raise BNGValueError(data['bngValueError'])
    return data


def angle_to_quat(angle):
    """
    Converts an euler angle to a quaternion.

    Args:
        angle (tuple): Euler angle (degrees)

    Return:
        Quaterion with the order (x, y, z, w) with w representing the real
        component
    """
    angle = np.radians(angle)

    cy = np.cos(angle[2] * 0.5)
    sy = np.sin(angle[2] * 0.5)
    cp = np.cos(angle[1] * 0.5)
    sp = np.sin(angle[1] * 0.5)
    cr = np.cos(angle[0] * 0.5)
    sr = np.sin(angle[0] * 0.5)

    w = cr * cp * cy + sr * sp * sy
    x = sr * cp * cy - cr * sp * sy
    y = cr * sp * cy + sr * cp * sy
    z = cr * cp * sy - sr * sp * cy

    return (x, y, z, w)


def compute_rotation_matrix(quat):
    """
    Calculates the rotation matrix for the given quaternion
    to be used in a scenario prefab.

    Args:
        quat (tuple): Quaterion with the order (x, y, z, w) with w
                      representing the real component

    Return:
        The rotation matrix as np array.
    """
    norm = np.linalg.norm(quat)
    eps = np.finfo(float).eps
    if np.abs(norm-1) > eps:
        quat /= norm
    x, y, z, w = quat[0], quat[1], quat[2], quat[3]
    rot_mat = np.array([
                            [1-2*(y**2+z**2), 2*(x*y-z*w), 2*(x*z+y*w)],
                            [2*(x*y+z*w), 1-2*(x**2+z**2), 2*(y*z-x*w)],
                            [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x**2+y**2)]
                        ], dtype=float)
    return rot_mat


def quat_as_rotation_mat_str(quat):
    """
    For a given quaternion, the function computes the corresponding rotation
    matrix and converts it into a string.

    Args:
        quat (tuple): Quaterion with the order (x, y, z, w) with w
        representing the real component

    Return:
        Rotation matrix as string

    """
    mat = compute_rotation_matrix(quat)
    mat = mat.reshape(9).astype(str)
    return ' '.join(mat)
