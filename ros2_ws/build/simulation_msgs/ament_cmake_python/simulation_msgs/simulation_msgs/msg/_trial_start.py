# generated from rosidl_generator_py/resource/_idl.py.em
# with input from simulation_msgs:msg/TrialStart.idl
# generated code does not contain a copyright notice


# Import statements for member types

import builtins  # noqa: E402, I100

import math  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_TrialStart(type):
    """Metaclass of message 'TrialStart'."""

    _CREATE_ROS_MESSAGE = None
    _CONVERT_FROM_PY = None
    _CONVERT_TO_PY = None
    _DESTROY_ROS_MESSAGE = None
    _TYPE_SUPPORT = None

    __constants = {
    }

    @classmethod
    def __import_type_support__(cls):
        try:
            from rosidl_generator_py import import_type_support
            module = import_type_support('simulation_msgs')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'simulation_msgs.msg.TrialStart')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__msg__trial_start
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__msg__trial_start
            cls._CONVERT_TO_PY = module.convert_to_py_msg__msg__trial_start
            cls._TYPE_SUPPORT = module.type_support_msg__msg__trial_start
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__msg__trial_start

            from geometry_msgs.msg import Pose
            if Pose.__class__._TYPE_SUPPORT is None:
                Pose.__class__.__import_type_support__()

            from geometry_msgs.msg import PoseArray
            if PoseArray.__class__._TYPE_SUPPORT is None:
                PoseArray.__class__.__import_type_support__()

            from std_msgs.msg import Header
            if Header.__class__._TYPE_SUPPORT is None:
                Header.__class__.__import_type_support__()

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class TrialStart(metaclass=Metaclass_TrialStart):
    """Message class 'TrialStart'."""

    __slots__ = [
        '_header',
        '_trial_name',
        '_trial_number',
        '_spawn',
        '_target',
        '_people',
        '_time_limit',
    ]

    _fields_and_field_types = {
        'header': 'std_msgs/Header',
        'trial_name': 'string',
        'trial_number': 'uint16',
        'spawn': 'geometry_msgs/Pose',
        'target': 'geometry_msgs/Pose',
        'people': 'geometry_msgs/PoseArray',
        'time_limit': 'double',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.NamespacedType(['std_msgs', 'msg'], 'Header'),  # noqa: E501
        rosidl_parser.definition.UnboundedString(),  # noqa: E501
        rosidl_parser.definition.BasicType('uint16'),  # noqa: E501
        rosidl_parser.definition.NamespacedType(['geometry_msgs', 'msg'], 'Pose'),  # noqa: E501
        rosidl_parser.definition.NamespacedType(['geometry_msgs', 'msg'], 'Pose'),  # noqa: E501
        rosidl_parser.definition.NamespacedType(['geometry_msgs', 'msg'], 'PoseArray'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        from std_msgs.msg import Header
        self.header = kwargs.get('header', Header())
        self.trial_name = kwargs.get('trial_name', str())
        self.trial_number = kwargs.get('trial_number', int())
        from geometry_msgs.msg import Pose
        self.spawn = kwargs.get('spawn', Pose())
        from geometry_msgs.msg import Pose
        self.target = kwargs.get('target', Pose())
        from geometry_msgs.msg import PoseArray
        self.people = kwargs.get('people', PoseArray())
        self.time_limit = kwargs.get('time_limit', float())

    def __repr__(self):
        typename = self.__class__.__module__.split('.')
        typename.pop()
        typename.append(self.__class__.__name__)
        args = []
        for s, t in zip(self.__slots__, self.SLOT_TYPES):
            field = getattr(self, s)
            fieldstr = repr(field)
            # We use Python array type for fields that can be directly stored
            # in them, and "normal" sequences for everything else.  If it is
            # a type that we store in an array, strip off the 'array' portion.
            if (
                isinstance(t, rosidl_parser.definition.AbstractSequence) and
                isinstance(t.value_type, rosidl_parser.definition.BasicType) and
                t.value_type.typename in ['float', 'double', 'int8', 'uint8', 'int16', 'uint16', 'int32', 'uint32', 'int64', 'uint64']
            ):
                if len(field) == 0:
                    fieldstr = '[]'
                else:
                    assert fieldstr.startswith('array(')
                    prefix = "array('X', "
                    suffix = ')'
                    fieldstr = fieldstr[len(prefix):-len(suffix)]
            args.append(s[1:] + '=' + fieldstr)
        return '%s(%s)' % ('.'.join(typename), ', '.join(args))

    def __eq__(self, other):
        if not isinstance(other, self.__class__):
            return False
        if self.header != other.header:
            return False
        if self.trial_name != other.trial_name:
            return False
        if self.trial_number != other.trial_number:
            return False
        if self.spawn != other.spawn:
            return False
        if self.target != other.target:
            return False
        if self.people != other.people:
            return False
        if self.time_limit != other.time_limit:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def header(self):
        """Message field 'header'."""
        return self._header

    @header.setter
    def header(self, value):
        if __debug__:
            from std_msgs.msg import Header
            assert \
                isinstance(value, Header), \
                "The 'header' field must be a sub message of type 'Header'"
        self._header = value

    @builtins.property
    def trial_name(self):
        """Message field 'trial_name'."""
        return self._trial_name

    @trial_name.setter
    def trial_name(self, value):
        if __debug__:
            assert \
                isinstance(value, str), \
                "The 'trial_name' field must be of type 'str'"
        self._trial_name = value

    @builtins.property
    def trial_number(self):
        """Message field 'trial_number'."""
        return self._trial_number

    @trial_number.setter
    def trial_number(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'trial_number' field must be of type 'int'"
            assert value >= 0 and value < 65536, \
                "The 'trial_number' field must be an unsigned integer in [0, 65535]"
        self._trial_number = value

    @builtins.property
    def spawn(self):
        """Message field 'spawn'."""
        return self._spawn

    @spawn.setter
    def spawn(self, value):
        if __debug__:
            from geometry_msgs.msg import Pose
            assert \
                isinstance(value, Pose), \
                "The 'spawn' field must be a sub message of type 'Pose'"
        self._spawn = value

    @builtins.property
    def target(self):
        """Message field 'target'."""
        return self._target

    @target.setter
    def target(self, value):
        if __debug__:
            from geometry_msgs.msg import Pose
            assert \
                isinstance(value, Pose), \
                "The 'target' field must be a sub message of type 'Pose'"
        self._target = value

    @builtins.property
    def people(self):
        """Message field 'people'."""
        return self._people

    @people.setter
    def people(self, value):
        if __debug__:
            from geometry_msgs.msg import PoseArray
            assert \
                isinstance(value, PoseArray), \
                "The 'people' field must be a sub message of type 'PoseArray'"
        self._people = value

    @builtins.property
    def time_limit(self):
        """Message field 'time_limit'."""
        return self._time_limit

    @time_limit.setter
    def time_limit(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'time_limit' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'time_limit' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._time_limit = value
