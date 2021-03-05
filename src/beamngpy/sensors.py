"""
.. module:: sensors
    :platform: Windows
    :synopsis: Module containing the various sensors one can attach to a
               vehicle.

.. moduleauthor:: Marc Müller <mmueller@beamng.gmbh>
.. moduleauthor:: Pascale Maul <pmaul@beamng.gmbh>
.. moduleauthor:: Sedonas <https://github.com/Sedonas>

This module implements various sensors that can be attached to vehicles to
extract data from simulations.
"""
import base64
import logging as log
import mmap
import os
import sys

from abc import ABC, abstractmethod
from xml.dom import minidom
from xml.etree import ElementTree
from xml.etree.ElementTree import Element, SubElement

import numpy as np
from PIL import Image, ImageDraw, ImageFont

from .beamngcommon import BNGValueError

NEAR = 0.01
FAR = 1000

LIDAR_POINTS = 2000000


class AbstractSensor(ABC):
    """
    Abstract Sensor class declaring properties common to the ordinary and noise
    sensors.
    """

    @property
    @abstractmethod
    def data(self):
        pass

    @data.setter
    @abstractmethod
    def data(self, data):
        pass

    @data.deleter
    @abstractmethod
    def data(self):
        pass


class Sensor(AbstractSensor):
    """
    Sensor meta-class declaring methods common to them.
    """

    def __init__(self):
        self._data = dict()

    @property
    def data(self):
        """
        Property used to store sensor readings.
        """
        return self._data

    @data.setter
    def data(self, data):
        self._data = data

    @data.deleter
    def data(self):
        self._data = None

    def attach(self, vehicle, name):
        """
        Called when the sensor is attached to a :class:`.Vehicle` instance.
        Used to perform sensor setup code before the simulation is started.
        This is called *after* the sensor has been entered into the vehicle's
        map of sensors under the given name.

        Args:
            vehicle (:class:`.Vehicle`): The vehicle instance the sensor is
                                         being attached to.
            name (str): The name the sensor is known under to the vehicle.
        """
        pass

    def detach(self, vehicle, name):
        """
        Called when the sensor is detached from a :class:`.Vehicle` instance.
        Used to perform sensor teardown code after the simulation is finished.
        This is called *after* the sensor has been removed from the
        vehicle's map of sensors under the given name.

        Args:
            vehicle (:class:`.Vehicle`): The vehicle instance the sensor is
                                         being detached from.
            name (str): The name the sensor was known under to the vehicle.
        """
        pass

    def encode_engine_request(self):
        """
        Called to retrieve this sensor's data request to the engine as a
        dictionary. The dictionary returned by this method will be bundled
        along the vehicle's other sensors' requests as a SensorRequest to the
        game's engine.

        Note:
            Sensors require corresponding code in the simulator to handle
            requests.

        Example:
            Consult the implementation of the :class:`.Camera` sensor for a
            good example of an engine request.

        Returns:
            The request to send to the engine as a dictionary.
        """
        return None

    def encode_vehicle_request(self):
        """
        Called to retrieve this sensor's request to the vehicle as a
        dictionary. The dictionary returned by this method will be bundled
        along the vehicle's other sensors' requests as a SensorRequest to the
        attached vehicle.

        Note:
            Sensors require corresponding code in the simulator to handle
            requests.

        Example:
            Consult the implementation of the :class:`.Electrics` sensor for a
            good example of a vehicle request.

        Returns:
            The request to send to the vehicle as a dictionary.
        """
        return None

    def decode_response(self, resp):
        """
        Called to do post-processing on sensor data obtained from the
        simulation. This method is called after raw simulation data is received
        and the resulting processed data is considered the result of a sensor
        request.

        Example:
            Consult the implementation of the :class:`.Camera` sensor for a
            good example of decoding sensor data.
        """
        return resp

    def connect(self, bng, vehicle):
        """
        Called when the attached vehicle is being initialised in the
        simulation. This method is used to perform setup code that requires the
        simulation to be running.
        """
        pass

    def disconnect(self, bng, vehicle):
        """
        Called when the attached vehicle is being removed from simulation. This
        method is used to perform teardown code after the simulation.
        """
        pass

    def get_engine_flags(self):
        """
        Called to retrieve a dictionary of settings in the engine this sensor
        requires.

        Returns:
            A dictionary of flags to set in the engine for this sensor to
            function.
        """
        return dict()


class Camera(Sensor):
    """
    A camera sensor provides several types of image data from a user-defined
    perspective relative to the vehicle. It can provide the following types of
    data:

    * Colour images
    * Pixel-wise depth
    * Pixel-wise object annotation
    * Pixel-wise instance annotation to separate overlapping objects of the
      same type

    A single camera sensor can be configured to provide any or all of these
    data at once, ensuring they all align to the same perspective.
    """

    @staticmethod
    def extract_bboxes(semantic, instance, classes):
        """
        Analyzes the given semantic annotation and instance annotation images
        for its object bounding boxes. The identified objects are returned as
        a list of dictionaries containing their bounding box corners, class
        of object according to the corresponding color in the semantic
        annotations and the provided class mapping, and the color of the
        object in the instance annotation.

        Args:
            semantic (:class:`.Image`): The image containing semantic
                                        annotation information
            instance (:class:`.Image`): The image containing instance
                                        annotation information
            classes (dict): A mapping of colors to their class names to
                            identify object types based on the semantic
                            annotation information. The keys in this dictionary
                            are the respective colors expressed as a 24bit
                            integer, i.e. r * 256^2 + g * 256 + b.

        Returns:
            A list of bounding boxes specified as dictionaries. One example
            bounding box dictionary contains:

            ```
            {
                'bbox': [minx, miny, maxx, maxy],
                'color': [233, 11, 15],
                'class': ['CAR'],
            }
            ```

            Where minx, miny, maxx, maxy specify the corners of the bounding
            box, color contains the RGB color of the object in the instance
            annotations, and class the object type identified through the
            given class mapping.
        """
        semantic = np.array(semantic)
        instance = np.array(instance)

        if semantic.shape != instance.shape:
            raise BNGValueError('Provided semantic and instance annotation '
                                'images have different sizes.')

        bboxes = {}

        for y in range(semantic.shape[0]):
            for x in range(semantic.shape[1]):
                color = instance[y, x]
                color_key = color[0] * 256 * 256 + color[1] * 256 + color[2]
                if color_key == 0:
                    continue

                clazz = semantic[y, x]
                clazz_key = clazz[0] * 256 * 256 + clazz[1] * 256 + clazz[2]

                if classes[clazz_key] == 'BACKGROUND':
                    continue

                if color_key in bboxes:
                    entry = bboxes[color_key]
                    bbox = entry['bbox']
                    bbox[0] = min(bbox[0], x)
                    bbox[1] = min(bbox[1], y)
                    bbox[2] = max(bbox[2], x)
                    bbox[3] = max(bbox[3], y)
                else:
                    entry = {
                        'bbox': [x, y, x, y],
                        'class': classes[clazz_key],
                        'color': [*color],
                    }
                    bboxes[color_key] = entry

        ret = []
        for _, v in bboxes.items():
            ret.append(v)
        return ret

    @staticmethod
    def export_bbox_xml(bboxes, folder=None, filename=None, path=None,
                        database=None, size=None):
        """
        Exports the given list of bounding boxes to the Pascal-VOC XML
        standard. The bounding box list is expected to be in the format that
        `extract_bboxes` returns. Additional properties to this function
        correspond to tags in the Pascal-VOC standard.

        Args:
            bboxes (list): The list of bounding boxes the export. Consult the
                           documentation of `extract_bboxes` for information
                           on its expected structure.
            folder (str): Contents of the `<folder>` tag. Optional.
            filename (str): Contents of the `<filename>` tag. Optional.
            path (str): Contents of the `<path>` tag. Optional.
            database (str): Contents of the `<database>` tag. Optional.
            size (tuple): Contents of the `<size>` tag. It's expected to be a
                          tuple of the image width, height, and depth. Optional.

        Returns:
            XML string encoding the given list of bounding boxes according to
            Pascal-VOC.
        """
        root = Element('annotation')

        if folder:
            folder_elem = SubElement(root, 'folder')
            folder_elem.text = folder

        if filename:
            file_elem = SubElement(root, 'filename')
            file_elem.text = filename

        if path:
            path_elem = SubElement(root, 'path')
            path_elem.text = path

        if database:
            source = SubElement(root, 'source')
            database_elem = SubElement(source, 'database')
            database_elem.text = database

        if size:
            size_elem = SubElement(root, 'size')

            width = SubElement(size_elem, 'width')
            width.text = str(size[0])

            height = SubElement(size_elem, 'height')
            height.text = str(size[1])

            depth = SubElement(size_elem, 'depth')
            depth.text = str(size[2])

        segmented = SubElement(root, 'segmented')
        segmented.text = '0'

        for idx, bbox in enumerate(bboxes):
            object_elem = SubElement(root, 'object')
            name = SubElement(object_elem, 'name')
            name.text = '{}_{}'.format(bbox['class'], idx)

            pose = SubElement(object_elem, 'pose')
            pose.text = 'Unspecified'

            truncated = SubElement(object_elem, 'truncated')
            truncated.text = '0'

            difficult = SubElement(object_elem, 'difficult')
            difficult.text = '0'

            bndbox = SubElement(object_elem, 'bndbox')

            xmin = SubElement(bndbox, 'xmin')
            xmin.text = str(bbox['bbox'][0])

            ymin = SubElement(bndbox, 'ymin')
            ymin.text = str(bbox['bbox'][1])

            xmax = SubElement(bndbox, 'xmax')
            xmax.text = str(bbox['bbox'][2])

            ymax = SubElement(bndbox, 'ymax')
            ymax.text = str(bbox['bbox'][3])

        ret = ElementTree.tostring(root, 'utf-8')
        ret = minidom.parseString(ret)
        return ret.toprettyxml(indent='  ')

    @staticmethod
    def draw_bboxes(bboxes, color, width=3, font='arial.ttf', fontsize=14):
        """
        Draws the given list of bounding boxes onto the given image. The boxes
        are drawn with the given width of outlines in pixels and the given font
        and size configuration. The given image is not directly modified and
        the boxes are drawn onto a copy. The bboxes are expected to be in the
        format returned by `extract_bboxes`.

        Args:
            bboxes (list): List of bounding boxes to draw. Consult the
                           documentation of `extract_bboxes` for information
                           on the structure of this list.
            color (:class:`.Image`): The image to draw bounding boxes on. Will
                                     be copied and not modified in place.
            width (int): The width of bounding box outlines in pixels.
            font (str): A string specifying the font bounding box labels are
                        supposed to have.
            fontsize (int): The font size to draw labels with.

        Returns:
            A `:class:`.Image` that is a copy of the given image with bounding
            boxes drawn onto it.
        """

        color = color.copy()
        draw = ImageDraw.Draw(color)

        font = ImageFont.truetype(font=font, size=fontsize)

        for idx, bbox in enumerate(bboxes):
            bbox_color = bbox['color']
            bbox_color = (bbox_color[0], bbox_color[1], bbox_color[2])
            bbox_corners = bbox['bbox']
            draw.rectangle(bbox_corners, outline=bbox_color, width=width)

            text = '{}_{}'.format(bbox['class'], idx)
            text_pos = (bbox_corners[0], bbox_corners[3])
            text_anchor = 'lt'

            if text_pos[1] > color.size[1] * 0.9:
                text_pos = (bbox_corners[0], bbox_corners[1])
                text_anchor = 'lb'

            draw.text(text_pos, text, fill='#FFFFFF', stroke_width=2,
                      stroke_fill='#000000', font=font, anchor=text_anchor)

        return color

    def __init__(self, pos, direction, fov, resolution, near_far=(NEAR, FAR),
                 colour=False, depth=False, annotation=False, instance=False,
                 shmem=True):
        """
        The camera sensor is set up with a fixed offset position and
        directional vector to face relative to the vehicle. This means as the
        vehicle moves and rotates, the camera is moved and rotated accordingly.

        Besides position and orientation, the image can further be customised
        with the FoV angle the camera should have, the resolution of the
        image(s) it outputs, and the near/far plane at which objects get
        clipped from view.

        Which sensor data to provide can be indicated using boolean flags for
        the corresponding type. The way data is transmitted can be chosen via
        the `shmem` flag. When true, the data is exchanged via a region of
        memory shared with the simulator. This is a lot faster, but has the
        downside of requiring the Python process and the simulator to be on the
        same machine. When false, data is sent via a socket and thus possible
        to be transmitted over network.

        Args:
            pos (tuple): (x,y,z) tuple of the camera's position offset relative
                         to the vehicle it's attached to.
            direction (tuple): (x,y,z) tuple expressing the direction vector
                               the camera is facing.
            fov (float): The Field of View of the camera.
            resolution (tuple): (width,height) tuple encoding the camera's
                                output resolution.
            near_far (tuple): (near,far) tuple of the distance below which and
                              after which geometry gets clipped. Usually
                              does not need to be changed.
            colour (bool): Whether to output colour information.
            depth (bool): Whether to output depth information.
            annotation (bool): Whether to output annotation information.
            instance (bool): Whether to output instance annotation information.
            shmem (bool): Whether to use shared memory for sensor data
                          transmission.
        """
        super().__init__()
        self.pos = pos
        self.direction = direction
        self.fov = fov
        self.resolution = resolution
        self.near_far = near_far

        self.colour = colour
        self.depth = depth
        self.annotation = annotation
        self.instance = instance

        self.shmem = shmem
        self.colour_handle = None
        self.colour_shmem = None
        self.depth_handle = None
        self.depth_shmem = None
        self.annotation_handle = None
        self.annotation_shmem = None
        self.instance_handle = None
        self.instance_shmem = None

    def attach(self, vehicle, name):
        """
        This method is called when the camera is attached and allocates
        shared memory space to house the sensor data the camera is supposed
        to provide.

        Args:
            vehicle (:class:`.Vehicle`): The vehicle the camera is being
                                         attached to.
            name (str): The name of the camera.
        """
        if self.shmem:
            pid = os.getpid()
            prefix = ''
            if vehicle:
                prefix = vehicle.vid
            size = self.resolution[0] * self.resolution[1] * 4
            self.colour_handle = '{}.{}.{}.colour'
            self.colour_handle = self.colour_handle.format(pid, prefix, name)
            self.colour_shmem = mmap.mmap(0, size, self.colour_handle)
            log.debug('Bound shmem for colour: %s', self.colour_handle)

            self.depth_handle = '{}.{}.{}.depth'
            self.depth_handle = self.depth_handle.format(pid, prefix, name)
            self.depth_shmem = mmap.mmap(0, size, self.depth_handle)
            log.debug('Bound shmem for depth: %s', self.depth_handle)

            self.annotation_handle = '{}.{}.{}.annotate'
            self.annotation_handle = self.annotation_handle.format(pid, prefix,
                                                                   name)
            self.annotation_shmem = mmap.mmap(0, size, self.annotation_handle)
            log.debug('Bound shmem for annotation: %s', self.annotation_handle)

            self.instance_handle = '{}.{}.{}.instance'
            self.instance_handle = self.instance_handle.format(pid, prefix,
                                                               name)
            self.instance_shmem = mmap.mmap(0, size, self.instance_handle)
            log.debug('Bound shmem for instance: %s', self.instance_handle)

    def detach(self, vehicle, name):
        """
        This method is called when the camera is detached from the vehicle. It
        de-allocates the shared memory space obtained for sensor data.

        Args:
            vehicle (:class:`.Vehicle`): The vehicle the camera is being
                                         detached from.
            name (str): The name of the camera.
        """
        if self.colour_shmem:
            log.debug('Unbinding shmem for color: %s', self.colour_handle)
            self.colour_shmem.close()

        if self.depth_shmem:
            log.debug('Unbinding shmem for depth: %s', self.depth_handle)
            self.depth_shmem.close()

        if self.annotation_shmem:
            log.debug('Unbinding shmem for annotation: %s',
                      self.annotation_handle)
            self.annotation_shmem.close()

        if self.instance_shmem:
            log.debug('Unbinding shmem for instance: %s', self.instance_handle)
            self.instance_shmem.close()

    def connect(self, bng, vehicle):
        """
        This method is called when the vehicle is set up in the simulation.
        It's used to inform the simulation about the shared memory used to
        exchange sensor data for this camera.

        Args:
            bng (:class:`.BeamNGpy`): Running instance of BeamNGpy.
            vehicle (:class:`.Vehicle`): The vehicle being connected.
        """
        size = self.resolution[0] * self.resolution[1] * 4  # RGBA / L are 4bbp

        if self.colour_shmem:
            bng.open_shmem(self.colour_handle, size)

        if self.depth_shmem:
            bng.open_shmem(self.depth_handle, size)

        if self.annotation_shmem:
            bng.open_shmem(self.annotation_handle, size)

        if self.instance_shmem:
            bng.open_shmem(self.instance_handle, size)

    def disconnect(self, bng, vehicle):
        """
        This method is called when the vehicle is disconnected from the
        simulation. It's used to tell the simulation to close the shared memory
        used to exchange sensor data for this camera.

        Args:
            bng (:class:`.BeamNGpy`): Running instance of BeamNGpy.
            vehicle (:class:`.Vehicle`): The vehicle being disconnected.
        """
        if self.colour_shmem:
            bng.close_shmem(self.colour_handle)

        if self.depth_shmem:
            bng.close_shmem(self.depth_handle)

        if self.annotation_shmem:
            bng.close_shmem(self.annotation_handle)

        if self.instance_shmem:
            bng.close_shmem(self.instance_handle)

    def encode_engine_request(self):
        """
        This method encodes a render request to the simulation engine along
        with the properties this camera is configured with.

        Returns:
            The request to the engine as a dictionary. This dictionary contains
            fields for each property of the requested render and which modes
            (colour, depth, annotation) to render in.
        """
        req = dict(type='Camera')

        if self.colour_shmem:
            req['color'] = self.colour_handle
        else:
            req['color'] = self.colour

        if self.depth_shmem:
            req['depth'] = self.depth_handle
        else:
            req['depth'] = self.depth

        if self.annotation_shmem:
            req['annotation'] = self.annotation_handle
        else:
            req['annotation'] = self.annotation

        if self.instance_shmem:
            req['instance'] = self.instance_handle
        else:
            req['instance'] = self.instance

        req['pos'] = [float(f) for f in self.pos]
        req['direction'] = [float(f) for f in self.direction]
        req['fov'] = self.fov
        req['resolution'] = [int(i) for i in self.resolution]
        req['near_far'] = [float(f) for f in self.near_far]
        req['shmem'] = self.shmem

        return req

    def decode_image(self, buffer, width, height, channels, dtype=np.uint8):
        img_d = base64.b64decode(buffer)
        img_d = np.frombuffer(img_d, dtype=dtype)
        if channels > 1:
            img_d = img_d.reshape(height, width, channels)
        else:
            img_d = img_d.reshape(height, width)
        return Image.fromarray(img_d)

    def decode_b64_response(self, resp):
        decoded = dict(type='Camera')
        img_w = resp['width']
        img_h = resp['height']

        if self.colour:
            decoded['colour'] = self.decode_image(resp['colorRGB8'],
                                                  img_w, img_h, 4)

        if self.annotation:
            decoded['annotation'] = self.decode_image(resp['annotationRGB8'],
                                                      img_w, img_h, 4)

        if self.instance:
            decoded['instance'] = self.decode_image(resp['instanceRGB8'],
                                                    img_w, img_h, 4)

        if self.depth:
            decoded['depth'] = self.decode_image(resp['depth32F'],
                                                 img_w, img_h, 1,
                                                 dtype=np.float32)

        return decoded

    def decode_shmem_response(self, resp):
        """
        This method obtains sensor data written to shared memory and decodes
        them as images. The resulting data is returned as a dictionary. This
        dictionary contains an entry for each requested image type that is
        mapped to a :class:`PIL.Image` instance.

        Args:
            resp (dict): The raw sensor data as a dictionary that was returned
                         by the simulation.

        Returns:
            The decoded response as a dictionary.
        """
        decoded = dict(type='Camera')
        img_w = resp['width']
        img_h = resp['height']

        size = img_w * img_h * 4

        if self.colour_shmem:
            if 'color' in resp.keys():
                self.colour_shmem.seek(0)
                colour_d = self.colour_shmem.read(size)
                colour_d = np.frombuffer(colour_d, dtype=np.uint8)
                colour_d = colour_d.reshape(img_h, img_w, 4)
                decoded['colour'] = Image.fromarray(colour_d)
            else:
                print('Color buffer failed to render. Check that you '
                      'aren\'t running on low settings.', file=sys.stderr)

        if self.annotation_shmem:
            if 'annotation' in resp.keys():
                self.annotation_shmem.seek(0)
                annotate_d = self.annotation_shmem.read(size)
                annotate_d = np.frombuffer(annotate_d, dtype=np.uint8)
                annotate_d = annotate_d.reshape(img_h, img_w, 4)
                decoded['annotation'] = Image.fromarray(annotate_d)
            else:
                print('Annotation buffer failed to render. Check that you '
                      'aren\'t running on low settings.', file=sys.stderr)

        if self.depth_shmem:
            if 'depth' in resp.keys():
                self.depth_shmem.seek(0)
                depth_d = self.depth_shmem.read(size)
                depth_d = np.frombuffer(depth_d, dtype=np.float32)
                depth_d = depth_d / FAR
                depth_d = depth_d.reshape(img_h, img_w)
                depth_d = np.uint8(depth_d * 255)
                decoded['depth'] = Image.fromarray(depth_d)
            else:
                print('Depth buffer failed to render. Check that you '
                      'aren\'t running on low settings.', file=sys.stderr)

        if self.instance_shmem:
            if 'instance' in resp.keys():
                self.instance_shmem.seek(0)
                instance_d = self.instance_shmem.read(size)
                instance_d = np.frombuffer(instance_d, dtype=np.uint8)
                instance_d = instance_d.reshape(img_h, img_w, 4)
                decoded['instance'] = Image.fromarray(instance_d)
            else:
                print('Instance buffer failed to render. Check that you '
                      'aren\'t running on low settings.', file=sys.stderr)

        return decoded

    def decode_response(self, resp):
        if self.shmem:
            return self.decode_shmem_response(resp)
        else:
            return self.decode_b64_response(resp)

    def get_engine_flags(self):
        """
        Called to retrieve settings for the simulation engine. Depending on the
        types of data this camera is supposed to provide, this method returns
        a dictionary enabling certain render modes in the engine.
        """
        flags = dict()
        if self.annotation:
            flags['annotations'] = True
        return flags


class Lidar(Sensor):
    max_points = LIDAR_POINTS
    shmem_size = LIDAR_POINTS * 3 * 4

    def __init__(self, offset=(0, 0, 1.7), direction=(0, -1, 0), vres=32,
                 vangle=26.9, rps=2200000, hz=20, angle=360, max_dist=200,
                 visualized=True, shmem=True):
        """
        The Lidar sensor provides 3D point clouds representing the environment
        as detected by a pulsing laser emitted from the vehicle. The range,
        position, and refresh rate of this sensor can be customised.
        """
        super().__init__()
        self.use_shmem = shmem
        self.handle = None
        self.shmem = None

        self.offset = offset
        self.direction = direction

        self.vres = vres
        self.vangle = vangle
        self.rps = rps
        self.hz = hz
        self.angle = angle
        self.max_dist = max_dist

        self.visualized = visualized

    def attach(self, vehicle, name):
        """
        Called when the lidar sensor is attached to a vehicle. This method
        allocates shared memory space to exchange lidar data with the engine.

        Args:
            vehicle (:class:`.Vehicle`): The vehicle the sensor is being
                                         attached to.
            name (str): The name of the sensor.
        """
        pid = os.getpid()
        self.handle = '{}.{}.{}.lidar'.format(pid, vehicle.vid, name)
        if self.use_shmem:
            self.shmem = mmap.mmap(0, Lidar.shmem_size, self.handle)
            log.debug('Bound memory for lidar: %s', self.handle)

    def detach(self, vehicle, name):
        """
        Called when the lidar sensor is detached from a vehicle. This method
        de-allocates the shared memory used to exchange lidar data with the
        engine.

        Args:
            vehicle (:class:`.Vehicle`): The vehicle the sensor is being
                                         detached from.
            name (str): The name of the sensor.
        """
        if self.use_shmem:
            self.shmem.close()

    def connect(self, bng, vehicle):
        if self.use_shmem:
            bng.open_lidar(self.handle, vehicle, self.handle, Lidar.shmem_size,
                           offset=self.offset, direction=self.direction,
                           vres=self.vres, vangle=self.vangle, rps=self.rps,
                           hz=self.hz, angle=self.angle, max_dist=self.max_dist,
                           visualized=self.visualized)
        else:
            bng.open_lidar(self.handle, vehicle, '', 0, offset=self.offset,
                           direction=self.direction, vres=self.vres,
                           vangle=self.vangle, rps=self.rps, hz=self.hz,
                           angle=self.angle, max_dist=self.max_dist,
                           visualized=self.visualized)

    def disconnect(self, bng, vehicle):
        bng.close_lidar(self.handle)

    def encode_engine_request(self):
        """
        Called to obtain the engine request for this lidar sensor. Encodes the
        properties of this lidar to obtain data according to them.

        Returns:
            The engine request containing the settings of this lidar sensor as
            a dictionary.
        """
        req = dict(type='Lidar')
        req['name'] = self.handle
        req['shmem'] = self.use_shmem
        return req

    def decode_response(self, resp):
        """
        Reads the raw point cloud the simulation wrote to the shared memory and
        creates a numpy array of points from them. The recoded response is
        returned as a dictionary with the numpy array in the ``points`` entry.

        Returns:
            The decoded response as a dictionary with the point cloud as a
            numpy array in the ``points`` entry. The numpy array is a linear
            sequence of coordinate triplets in the form of [x0, y0, z0, x1,
            y1, z1, ..., xn, yn, zn].
        """
        points_buf = None

        if self.use_shmem:
            size = resp['size']
            self.shmem.seek(0)
            points_buf = self.shmem.read(size)
            points_buf = np.frombuffer(points_buf, dtype=np.float32)
        else:
            points_buf = resp['points']
            points_buf = np.array(points_buf, dtype=np.float32)

        assert points_buf.size % 3 == 0
        resp = dict(type='Lidar')
        resp['points'] = points_buf
        return resp

    def get_engine_flags(self):
        """
        Returns: a dictionary with the engine flag to enable Lidar.
        """
        flags = dict(lidar=True)
        return flags


class GForces(Sensor):
    """
    This sensor is used to obtain the GForces acting on a vehicle.

    # TODO: GForce sensor for specific points on/in the vehicle
    """

    def __init__(self):
        super().__init__()

    def encode_vehicle_request(self):
        req = dict(type='GForces')
        return req


class Electrics(Sensor):
    """
    This sensor is used to retrieve various values made available by the car's
    eletrics systems. These values include:

    # TODO: List all the electrics.lua values.
    - abs (int): ABS state
    - abs_active (bool):
    - airspeed (float): Airspeed
    - airflowspeed (float):
    - altitude (float): Z axis position
    - avg_wheel_av (float):
    - brake (int): Brake value
    - brake_lights (int):
    - brake_input (int): Brake input value
    - check_engine (bool): Check engine light state.
    - clutch (int): Clutch value
    - clutch_input (int): Clutch input value
    - clutch_ratio (int):
    - driveshaft (float): Driveshaft
    - engine_load (float):
    - engine_throttle (int): Engine throttle state
    - esc (int): ESC state. 0 = not present/inactive, 1 = disabled,
                 Blink = active
    - esc_active (bool):
    - exhaust_flow (float):
    - fog_lights (int): Fog light state
    - fuel (float): Percentage of fuel remaining.
    - fuel_capacity (int): Total Fuel Capacity [L].
    - fuel_volume (float):
    - gear (int):
    - gear_a (int): Gear selected in automatic mode.
    - gear_index (int):
    - gear_m (int): Gear selected in manual mode.
    - hazard (int): Hazard light state
    - hazard_signal (bool):
    - headlights (int):
    - highbeam (int): High beam state
    - horn (int):
    - ignition (bool): Engine state
    - left_signal (bool):
    - lightbar (int): Lightbar state
    - lights (int): General light state. 1 = low, 2 = high
    - lowbeam (int): Low beam state
    - lowfuel (bool): Low fuel indicator
    - lowhighbeam (int): Low-high beam state
    - lowpressure (int): Low fuel pressure indicator
    - oil (int):
    - oil_temperature (float): Oil temperature [C].
    - parking (int): Parking lights on/off (not implemented yet)
    - parkingbrake (float): Parking brake state. 0.5 = halfway on
    - parkingbrake_input (int): Parking brake input state
    - radiator_fan_spin (int):
    - reverse (int): Reverse gear state
    - right_signal (bool):
    - rpm (float): Engine RPM
    - rpmspin (float):
    - rpm_tacho (float):
    - running (bool): Engine running state
    - signal_l (int): Left signal state. 0.5 = halfway to full blink
    - signal_r (int): Right signal state. 0.5 = halfway to full blink
    - steering (int): Steering state
    - steering_input (int): Steering input state
    - tcs (int): TCS state. 0 = not present/inactive, 1 = disabled,
                 Blink = active
    - tcs_active (bool):
    - throttle (int): Throttle state
    - throttle_factor (int):
    - throttle_input (int): Throttle input state
    - turnsignal (int): Turn signal value. -1 = Left, 1 = Right,
                        gradually 'fades' between values. Use "signal_L" and
                        "signal_R" for flashing indicators.
    - two_step (bool):
    - water_temperature (float): Water temperature [C].
    - wheelspeed (float): Wheel speed [m/s].
    """
    name_map = {
        'absActive': 'abs_active',
        'avgWheelAV': 'avg_wheel_av',
        'brakelights': 'brake_lights',
        'checkengine': 'check_engine',
        'clutchRatio': 'clutch_ratio',
        'engineLoad': 'engine_load',
        'engineThrottle': 'engine_throttle',
        'escActive': 'esc_active',
        'exhaustFlow': 'exhaust_flow',
        'fog': 'fog_lights',
        'fuelVolume': 'fuel_volume',
        'fuelCapacity': 'fuel_capacity',
        'gear_A': 'gear_a',
        'gearIndex': 'gear_index',
        'gear_M': 'gear_m',
        'hazard_enabled': 'hazard_signal',
        'isShifting': 'is_shifting',
        'lights_state': 'headlights',
        'oiltemp': 'oil_temperature',
        'radiatorFanSpin': 'radiator_fan_spin',
        'rpmTacho': 'rpm_tacho',
        'signal_L': 'signal_l',
        'signal_left_input': 'left_signal',
        'signal_R': 'signal_r',
        'signal_right_input': 'right_signal',
        'tcsActive': 'tcs_active',
        'throttleFactor': 'throttle_factor',
        'twoStep': 'two_step',
        'watertemp': 'water_temperature',
    }

    def __init__(self):
        super().__init__()

    def _rename_values(self, vals):
        """
        The values returned from the game often don't follow any naming
        convention and especially don't follow this library's, so we rename
        some of them here to be more consistent.
        """

        for k, v in Electrics.name_map.items():
            if k in vals:
                vals[v] = vals[k]
                del vals[k]
        return vals

    def _reassign_values(self, vals):
        if 'left_signal' in vals:
            vals['left_signal'] = vals['left_signal'] == 1
        if 'right_signal' in vals:
            vals['right_signal'] = vals['right_signal'] == 1
        if 'hazard_signal' in vals:
            vals['hazard_signal'] = vals['hazard_signal'] == 1
        return vals

    def encode_vehicle_request(self):
        req = dict(type='Electrics')
        return req

    def decode_response(self, resp):
        if 'values' in resp:
            ret = self._rename_values(resp['values'])
            ret = self._reassign_values(ret)
            return ret
        return None


class Damage(Sensor):
    """
    The damage sensor retrieves information about how damaged the structure
    of the vehicle is. It's important to realise that this is a sensor that has
    no analogue in real life as it returns a perfect knowledge overview of how
    deformed the vehicle is. It's therefore more of a ground truth than
    simulated sensor data.
    """

    def __init__(self):
        super().__init__()

    def encode_vehicle_request(self):
        req = dict(type='Damage')
        return req


class Timer(Sensor):
    """
    The timer sensor keeps track of the time that has passed since the
    simulation started. It provides that information in seconds relative to the
    scenario start and does not represent something like a day time or date. It
    properly handles pausing the simulation, meaning the value of the timer
    sensor does not progress while the simulation is paused.

    When polled, this sensor provides the time in seconds since the start of
    the scenario in a dictionary under the 'time' key.
    """

    def __init__(self):
        super().__init__()

    def encode_engine_request(self):
        req = dict(type='Timer')
        return req


class State(Sensor):
    """
    The state sensor monitors general stats of the vehicle, such as position,
    direction, velocity, etc. It is a default sensor every vehicle has and is
    used to update the vehicle.state attribute.
    """

    def __init__(self):
        super().__init__()
        self.connected = False

    def connect(self, bng, vehicle):
        self.connected = True

    def disconnect(self, bng, vehicle):
        self.connected = False

    def encode_vehicle_request(self):
        req = dict(type='State')
        return req

    def decode_response(self, resp):
        if 'state' in resp:
            return resp['state']

        return None


class IMU(Sensor):
    """
    An IMU measures forces and rotational acceleration at a certain point on a
    vehicle. This can be used to analyze forces acting on certain areas of the
    car (like the driver's position) or estimate the trajectory of a vehicle
    from its rotation and acceleration.
    """

    def __init__(self, pos=None, node=None, name=None, debug=False):
        super().__init__()

        if pos is not None and node is not None:
            raise BNGValueError('Cannot specify both position and node for '
                                'an IMU')

        self._pos = pos
        self._node = node

        self._name = name
        if self._name is None:
            self._name = str(hash(self))

        self._debug = debug

    def connect(self, bng, vehicle):
        if self._pos is not None:
            vehicle.add_imu_position(self._name, self._pos, self._debug)

        if self._node is not None:
            vehicle.add_imu_node(self._name, self._node, self._debug)

    def disconnect(self, bng, vehicle):
        vehicle.remove_imu(self._name)

    def encode_vehicle_request(self):
        req = dict(type='IMU')
        req['name'] = self._name
        return req


class Ultrasonic(Sensor):
    """
    An ultrasonic sensor (aka parking sensor) that can be placed at
    any point outside the vehicle.
    This is not an ideal sensor but one whose output is simulated based on
    depth information in images.
    """

    def __init__(self,
                 pos,
                 rot,
                 fov=(70, 35),
                 min_resolution=256,
                 near_far=(0.15, 5.5)):
        self.pos = pos
        self.rot = rot
        self.fov = fov[0]
        res_height = int(min_resolution/fov[0]*fov[1])
        self.resolution = (min_resolution, res_height)
        self.near_far = near_far
        self.vis_spec = None

    def encode_engine_request(self):
        req = dict(type='Ultrasonic')
        req['pos'] = self.pos
        req['rot'] = self.rot
        req['fov'] = self.fov
        req['resolution'] = self.resolution
        req['near_far'] = self.near_far
        return req

    def startVisualization(self, bng, vehicle_id, color, radius=.1):
        """
        Called, after opening BeamNG, this will start the visualization
        of a sphere at the sensor position. This functionality is intended
        for sensors attached to vehicles and not for world sensors.

        Args:
            bng(:class:`.BeamNGpy`): instance of BeamNGpy
            vehicle_id(string): ID of the vehicle the sensor belongs to
            color(tuple): four floats (RGBA) defining the color of the sphere
            radius(float): radius of the sphere
        """
        req = dict(type='StartUSSensorVisualization')
        req['vehicle'] = vehicle_id
        req['pos'] = self.pos
        req['rot'] = self.rot
        req['color'] = color
        req['radius'] = radius
        req['lineLength'] = self.near_far[1]
        bng.send(req)
        resp = bng.recv()
        self.vis_spec = resp['sphereID']

    def stopVisualization(self, bng):
        """
        Stops the sensor visualization.
        """
        if self.vis_spec is not None:
            data = dict(type='StopUSSensorVisualization')
            data['dynSphereID'] = self.vis_spec
            bng.send(data)
