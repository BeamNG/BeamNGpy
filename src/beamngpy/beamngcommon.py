"""
.. module:: beamngcommon
    :platform: Windows
    :synopsis: Module containing common functions used across various BeamNGpy
               modules.

.. moduleauthor:: Marc Müller <mmueller@beamng.gmbh>
"""

import logging as log
import json
import os
import socket

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
