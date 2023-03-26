import math
import copy
import numpy as np


class Position():
    def __init__(self, x=0, y=0, z=0):
        self.x = x
        self.y = y
        self.z = z

    def update(self, x=0, y=0, z=0):
        self.x = x
        self.y = y
        self.z = z


class Rotation():
    def __init__(self, roll=0, pitch=0, yaw=0):
        self.roll = roll
        self.pitch = pitch
        self.yaw = yaw

    def update(self, roll=0, pitch=0, yaw=0):
        self.roll = roll
        self.pitch = pitch
        self.yaw = yaw


class BoundingBox():
    # 1-------------2
    # |             |
    # |      o      |
    # |             |
    # 4-------------3
    #
    # AXIS:
    #
    # ^ y
    # |
    # |
    # |
    # |
    # |
    # .----------------> x
    def __init__(self, position, rotation, y, x):
        self.x_length = x
        self.y_length = y
        self.pts = self.calc_pts(position, rotation)

    def calc_pts(self, position, rotation):
        sin_yaw = math.sin(rotation.yaw)
        cos_yaw = math.cos(rotation.yaw)

        x_length_offset_x = self.x_length * cos_yaw / 2
        y_length_offset_x = self.y_length * sin_yaw / 2

        x_length_offset_y = self.x_length * sin_yaw / 2
        y_length_offset_y = self.y_length * cos_yaw / 2
        pos_1 = Position(x=position.x - x_length_offset_x - y_length_offset_x,
                         y=position.y - x_length_offset_y + y_length_offset_y)
        pos_2 = Position(x=position.x + x_length_offset_x - y_length_offset_x,
                         y=position.y + x_length_offset_y + y_length_offset_y)
        pos_3 = Position(x=position.x + x_length_offset_x + y_length_offset_x,
                         y=position.y + x_length_offset_y - y_length_offset_y)
        pos_4 = Position(x=position.x - x_length_offset_x + y_length_offset_x,
                         y=position.y - x_length_offset_y - y_length_offset_y)
        return pos_1, pos_2, pos_3, pos_4

    def update(self, position, rotation):
        self.pts = self.calc_pts(position, rotation)


class BoxObject():
    def __init__(self, width, length, position=Position(0, 0), rotation=Rotation(0, 0, 0)):
        super().__init__()
        self.width = width
        self.length = length
        self.position = position
        self.rotation = rotation
        self.box = BoundingBox(position, rotation, width, length)

    def update_pos(self, position, rotation):
        self.position = position
        self.rotation = rotation
        self.box.update(position, rotation)

    def unwind_pos(self):
        return np.array([point.x for point in self.box.pts]), \
               np.array([point.y for point in self.box.pts])

    def get_bbox(self):
        return self.unwind_pos()


class Transform():
    def __init__(self, position=Position(), rotation=Rotation()):
        self._position = position
        self._rotation = rotation

    @property
    def position(self):
        return self._position

    @position.setter
    def position(self, var):
        self._position = var

    @property
    def rotation(self):
        return self._rotation

    @rotation.setter
    def rotation(self, var):
        self._rotation = var

    def __eq__(self, other):
        return self._position == other.position and self._rotation == other.rotation

    def to_np_transform(self):
        return np.array([
            self._position.x,
            self._position.y,
            self._position.z,
            self._rotation.roll,
            self._rotation.pitch,
            self._rotation.yaw,
        ])

    def to_2d_matrix(self):
        matrix = np.identity(3)
        cos_yaw = np.cos(self._rotation.yaw)
        sin_yaw = np.sin(self._rotation.yaw)
        matrix[0, 0] = cos_yaw
        matrix[0, 1] = -sin_yaw
        matrix[1, 0] = sin_yaw
        matrix[1, 1] = cos_yaw
        matrix[0, 2] = self._position.x
        matrix[1, 2] = self._position.y
        return matrix

    def translate(self, x, y, in_place):
        matrix = np.identity(3)
        matrix[0, 2] = x
        matrix[1, 2] = y

        transformed_matrix = self.to_2d_matrix().dot(matrix)
        x = transformed_matrix[0, 2]
        y = transformed_matrix[1, 2]
        yaw = np.cosh(transformed_matrix[0, 0])

        if in_place:
            self._position.x = x
            self._position.y = y
            # self._rotation.yaw = yaw
        else:
            new_self = copy.deepcopy(self)
            new_self.position.x = x
            new_self.position.y = y
            new_self.position.yaw = yaw
            return new_self
