"""Parameter values retain their original Python types."""
from enum import IntEnum


class Parameter:
    class Type(IntEnum):
        NOT_SET = 0
        BOOL = 1
        INTEGER = 2
        DOUBLE = 3
        STRING = 4
        BYTE_ARRAY = 5
        BOOL_ARRAY = 6
        INTEGER_ARRAY = 7
        DOUBLE_ARRAY = 8
        STRING_ARRAY = 9

    def __init__(self, name, type_=None, value=None):
        self.name = name
        self.value = value
        inferred = self.type_from_value(value)
        if type_ is not None and self.Type(type_) != inferred:
            raise TypeError('Parameter type does not match its value: ' + name)
        self.type_ = inferred

    @classmethod
    def type_from_value(cls, value):
        scalar_types = {bool: cls.Type.BOOL, int: cls.Type.INTEGER,
                        float: cls.Type.DOUBLE, str: cls.Type.STRING}
        if value is None:
            return cls.Type.NOT_SET
        if type(value) in scalar_types:
            return scalar_types[type(value)]
        if isinstance(value, (bytes, bytearray)):
            return cls.Type.BYTE_ARRAY
        if isinstance(value, (list, tuple)):
            if not value:
                return cls.Type.BYTE_ARRAY
            first = cls.type_from_value(value[0])
            if any(cls.type_from_value(item) != first for item in value):
                raise TypeError('ROS parameters require homogeneous arrays')
            array_types = {cls.Type.BOOL: cls.Type.BOOL_ARRAY,
                           cls.Type.INTEGER: cls.Type.INTEGER_ARRAY,
                           cls.Type.DOUBLE: cls.Type.DOUBLE_ARRAY,
                           cls.Type.STRING: cls.Type.STRING_ARRAY}
            if first in array_types:
                return array_types[first]
        raise TypeError('Unsupported ROS parameter value: ' + repr(value))
