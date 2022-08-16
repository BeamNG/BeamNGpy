import base64
import mmap
import os
from logging import DEBUG, getLogger
from xml.dom import minidom
from xml.etree import ElementTree
from xml.etree.ElementTree import Element, SubElement

import numpy as np
from beamngpy.beamngcommon import LOGGER_ID, BNGValueError
from PIL import Image, ImageDraw, ImageFont

from .sensor import Sensor

NEAR = 0.01
FAR = 1000


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
                            integer, i.e. `r * 256^2 + g * 256 + b`.
        Returns:
            A list of bounding boxes specified as dictionaries. One example
            bounding box dictionary contains:
            .. code-block:: python
                'bbox': [minx, miny, maxx, maxy],
                'color': [233, 11, 15],
                'class': ['CAR'],
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
                          tuple of the image width, height, and depth.
                          Optional.
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
                 colour=False, depth=False, depth_distance=(NEAR, FAR),
                 depth_inverse=False, annotation=False, instance=False,
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
            depth_distance (tuple): (near,far) tuple of the distance range
                                    depth values should be mapped between.
                                    For example, a distance_scale of (10, 50)
                                    would mean geometry closer than 10 would
                                    be mapped to black and geometry further
                                    than 50 would be mapped to white. All
                                    distances in-between are interpolated
                                    accordingly.
            depth_inverse (bool): If true, depth values are inversed so that
                                  geometry that is closer is white instead
                                  of black, and geometry that is further is
                                  black instead of white. This is more
                                  typical behaviour of real-life stereo
                                  cameras that output depth images.
            annotation (bool): Whether to output annotation information.
            instance (bool): Whether to output instance annotation information.
            shmem (bool): Whether to use shared memory for sensor data
                          transmission.
        """
        super().__init__()
        self.logger = getLogger(f'{LOGGER_ID}.Camera')
        self.logger.setLevel(DEBUG)
        self.pos = pos
        self.direction = direction
        self.fov = fov
        self.resolution = resolution
        self.near_far = near_far

        self.colour = colour
        self.depth = depth
        self.depth_distance = depth_distance
        self.depth_inverse = depth_inverse
        self.annotation = annotation
        self.instance = instance
        id = ['colour', 'depth', 'annotation', 'instance']
        properties = [self.colour, self.depth, self.annotation, self.instance]
        properties = [id for id, prop in zip(id, properties) if properties]
        properties = ', '.join(properties)
        self.logger.debug(f'Set up camera to render {properties} images.')

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
            self.logger.debug('Initializing shared memory.')
            pid = os.getpid()
            prefix = ''
            if vehicle:
                prefix = vehicle.vid
            size = self.resolution[0] * self.resolution[1] * 4
            if self.colour:
                self.colour_handle = '{}.{}.{}.colour'
                self.colour_handle = self.colour_handle.format(pid, prefix, name)
                self.colour_shmem = mmap.mmap(0, size, self.colour_handle)
                self.logger.debug('Bound shmem for colour: {self.colour_handle}')

            if self.depth:
                self.depth_handle = '{}.{}.{}.depth'
                self.depth_handle = self.depth_handle.format(pid, prefix, name)
                self.depth_shmem = mmap.mmap(0, size, self.depth_handle)
                self.logger.debug(f'Bound shmem for depth: {self.depth_handle}')

            if self.annotation:
                self.annotation_handle = '{}.{}.{}.annotate'
                self.annotation_handle = self.annotation_handle.format(pid, prefix,
                                                                       name)
                self.annotation_shmem = mmap.mmap(0, size, self.annotation_handle)
                self.logger.debug('Bound shmem for annotation: '
                                  f'{self.annotation_handle}')

            if self.instance:
                self.instance_handle = '{}.{}.{}.instance'
                self.instance_handle = self.instance_handle.format(pid, prefix,
                                                                   name)
                self.instance_shmem = mmap.mmap(0, size, self.instance_handle)
                self.logger.debug('Bound shmem for instance: '
                                  f'{self.instance_handle}')

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
            self.logger.debug('Unbinding shmem for color: '
                              f'{self.colour_handle}')
            self.colour_shmem.close()

        if self.depth_shmem:
            self.logger.debug('Unbinding shmem for depth: '
                              f'{self.depth_handle}')
            self.depth_shmem.close()

        if self.annotation_shmem:
            self.logger.debug('Unbinding shmem for annotation: '
                              f'{self.annotation_handle}')
            self.annotation_shmem.close()

        if self.instance_shmem:
            self.logger.debug('Unbinding shmem for instance: '
                              f'{self.instance_handle}')
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
            req['color'] = None

        if self.depth_shmem:
            req['depth'] = self.depth_handle
        else:
            req['depth'] = None

        if self.annotation_shmem:
            req['annotation'] = self.annotation_handle
        else:
            req['annotation'] = None

        if self.instance_shmem:
            req['instance'] = self.instance_handle
        else:
            req['instance'] = None

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
            # TODO: More needs to be done here to scale the lightness values
            # between 0-255. Please see decode_shmem_response(). Currently
            # this would generate an invalid image where each pixel's value
            # would actually be a raw distance. I am unable to complete this
            # as using shmem=False currently does not work for me.

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
        img_w = int(resp['width'])
        img_h = int(resp['height'])

        size = img_w * img_h * 4

        if self.colour_shmem:
            if 'color' in resp.keys():
                self.colour_shmem.seek(0)
                colour_d = self.colour_shmem.read(size)
                colour_d = np.frombuffer(colour_d, dtype=np.uint8)
                colour_d = colour_d.reshape(img_h, img_w, 4)
                decoded['colour'] = Image.fromarray(colour_d)
            else:
                msg = 'Color buffer failed to render. '\
                      'Check that you aren\'t running on low settings.'
                self.logger.error(msg)

        if self.annotation_shmem:
            if 'annotation' in resp.keys():
                self.annotation_shmem.seek(0)
                annotate_d = self.annotation_shmem.read(size)
                annotate_d = np.frombuffer(annotate_d, dtype=np.uint8)
                annotate_d = annotate_d.reshape(img_h, img_w, 4)
                decoded['annotation'] = Image.fromarray(annotate_d)
            else:
                msg = 'Color buffer failed to render. '\
                      'Check that you aren\'t running on low settings.'
                self.logger.error(msg)

        if self.depth_shmem:
            if 'depth' in resp.keys():
                self.depth_shmem.seek(0)
                depth_d = self.depth_shmem.read(size)
                depth_d = np.frombuffer(depth_d, dtype=np.float32)
                # Use linear interpolation to map the depth values
                # between lightness values 0-255. Any distances outside
                # of the scale are clamped to either 0 or 255
                # respectively.
                if self.depth_inverse:
                    depth_dist = [self.depth_distance[0],
                                  self.depth_distance[1]]
                    depth_d = np.interp(depth_d,
                                        depth_dist,
                                        [255, 0],
                                        left=255,
                                        right=0)
                else:
                    depth_d = np.interp(depth_d, [self.depth_distance[0],
                                        self.depth_distance[1]], [0, 255],
                                        left=0,
                                        right=255)
                depth_d = depth_d.reshape(img_h, img_w)
                depth_d = np.uint8(depth_d)
                decoded['depth'] = Image.fromarray(depth_d)
            else:
                self.logger.error('Depth buffer failed to render. '
                                  'Check that you aren\'t running '
                                  'on low settings.')

        if self.instance_shmem:
            if 'instance' in resp.keys():
                self.instance_shmem.seek(0)
                instance_d = self.instance_shmem.read(size)
                instance_d = np.frombuffer(instance_d, dtype=np.uint8)
                instance_d = instance_d.reshape(img_h, img_w, 4)
                decoded['instance'] = Image.fromarray(instance_d)
            else:
                self.logger.error('Instance buffer failed to render. '
                                  'Check that you aren\'t running on low settings.')

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
