"""
.. module:: communication
    :platform: Windows
    :synopsis: Offers utility functions for socket communication via msgpack-
               encoded packets.
.. moduleauthor:: Marc Müller <mmueller@beamng.gmbh>

"""

import socket
import os

import msgpack


def send_msg(skt, data):
    data = msgpack.packb(data, use_bin_type=True, encoding='utf-8')
    length = '{:016}'.format(len(data))
    skt.send(bytes(length, 'ascii'))
    skt.send(data)


def recv_msg(skt):
    length = skt.recv(16)
    length = int(str(length, 'ascii'))
    data = skt.recv(length)
    data = msgpack.unpackb(data, encoding='utf-8')
    return data
