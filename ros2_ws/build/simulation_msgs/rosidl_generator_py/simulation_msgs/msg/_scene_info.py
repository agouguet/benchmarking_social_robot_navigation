# generated from rosidl_generator_py/resource/_idl.py.em
# with input from simulation_msgs:msg/SceneInfo.idl
# generated code does not contain a copyright notice


# Import statements for member types

import builtins  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_SceneInfo(type):
    """Metaclass of message 'SceneInfo'."""

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
                'simulation_msgs.msg.SceneInfo')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__msg__scene_info
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__msg__scene_info
            cls._CONVERT_TO_PY = module.convert_to_py_msg__msg__scene_info
            cls._TYPE_SUPPORT = module.type_support_msg__msg__scene_info
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__msg__scene_info

            from geometry_msgs.msg import Pose
            if Pose.__class__._TYPE_SUPPORT is None:
                Pose.__class__.__import_type_support__()

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


class SceneInfo(metaclass=Metaclass_SceneInfo):
    """Message class 'SceneInfo'."""

    __slots__ = [
        '_header',
        '_scenario_name',
        '_robot_start_pose',
        '_robot_target_pose',
        '_num_people',
        '_num_groups',
        '_environment',
    ]

    _fields_and_field_types = {
        'header': 'std_msgs/Header',
        'scenario_name': 'string',
        'robot_start_pose': 'geometry_msgs/Pose',
        'robot_target_pose': 'geometry_msgs/Pose',
        'num_people': 'uint16',
        'num_groups': 'uint16',
        'environment': 'string',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.NamespacedType(['std_msgs', 'msg'], 'Header'),  # noqa: E501
        rosidl_parser.definition.UnboundedString(),  # noqa: E501
        rosidl_parser.definition.NamespacedType(['geometry_msgs', 'msg'], 'Pose'),  # noqa: E501
        rosidl_parser.definition.NamespacedType(['geometry_msgs', 'msg'], 'Pose'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint16'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint16'),  # noqa: E501
        rosidl_parser.definition.UnboundedString(),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        from std_msgs.msg import Header
        self.header = kwargs.get('header', Header())
        self.scenario_name = kwargs.get('scenario_name', str())
        from geometry_msgs.msg import Pose
        self.robot_start_pose = kwargs.get('robot_start_pose', Pose())
        from geometry_msgs.msg import Pose
        self.robot_target_pose = kwargs.get('robot_target_pose', Pose())
        self.num_people = kwargs.get('num_people', int())
        self.num_groups = kwargs.get('num_groups', int())
        self.environment = kwargs.get('environment', str())

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
        if self.scenario_name != other.scenario_name:
            return False
        if self.robot_start_pose != other.robot_start_pose:
            return False
        if self.robot_target_pose != other.robot_target_pose:
            return False
        if self.num_people != other.num_people:
            return False
        if self.num_groups != other.num_groups:
            return False
        if self.environment != other.environment:
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
    def scenario_name(self):
        """Message field 'scenario_name'."""
        return self._scenario_name

    @scenario_name.setter
    def scenario_name(self, value):
        if __debug__:
            assert \
                isinstance(value, str), \
                "The 'scenario_name' field must be of type 'str'"
        self._scenario_name = value

    @builtins.property
    def robot_start_pose(self):
        """Message field 'robot_start_pose'."""
        return self._robot_start_pose

    @robot_start_pose.setter
    def robot_start_pose(self, value):
        if __debug__:
            from geometry_msgs.msg import Pose
            assert \
                isinstance(value, Pose), \
                "The 'robot_start_pose' field must be a sub message of type 'Pose'"
        self._robot_start_pose = value

    @builtins.property
    def robot_target_pose(self):
        """Message field 'robot_target_pose'."""
        return self._robot_target_pose

    @robot_target_pose.setter
    def robot_target_pose(self, value):
        if __debug__:
            from geometry_msgs.msg import Pose
            assert \
                isinstance(value, Pose), \
                "The 'robot_target_pose' field must be a sub message of type 'Pose'"
        self._robot_target_pose = value

    @builtins.property
    def num_people(self):
        """Message field 'num_people'."""
        return self._num_people

    @num_people.setter
    def num_people(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'num_people' field must be of type 'int'"
            assert value >= 0 and value < 65536, \
                "The 'num_people' field must be an unsigned integer in [0, 65535]"
        self._num_people = value

    @builtins.property
    def num_groups(self):
        """Message field 'num_groups'."""
        return self._num_groups

    @num_groups.setter
    def num_groups(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'num_groups' field must be of type 'int'"
            assert value >= 0 and value < 65536, \
                "The 'num_groups' field must be an unsigned integer in [0, 65535]"
        self._num_groups = value

    @builtins.property
    def environment(self):
        """Message field 'environment'."""
        return self._environment

    @environment.setter
    def environment(self, value):
        if __debug__:
            assert \
                isinstance(value, str), \
                "The 'environment' field must be of type 'str'"
        self._environment = value
