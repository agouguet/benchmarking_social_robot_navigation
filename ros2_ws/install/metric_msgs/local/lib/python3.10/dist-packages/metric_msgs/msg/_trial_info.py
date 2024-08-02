# generated from rosidl_generator_py/resource/_idl.py.em
# with input from metric_msgs:msg/TrialInfo.idl
# generated code does not contain a copyright notice


# Import statements for member types

import builtins  # noqa: E402, I100

import math  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_TrialInfo(type):
    """Metaclass of message 'TrialInfo'."""

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
            module = import_type_support('metric_msgs')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'metric_msgs.msg.TrialInfo')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__msg__trial_info
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__msg__trial_info
            cls._CONVERT_TO_PY = module.convert_to_py_msg__msg__trial_info
            cls._TYPE_SUPPORT = module.type_support_msg__msg__trial_info
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__msg__trial_info

            from builtin_interfaces.msg import Time
            if Time.__class__._TYPE_SUPPORT is None:
                Time.__class__.__import_type_support__()

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


class TrialInfo(metaclass=Metaclass_TrialInfo):
    """Message class 'TrialInfo'."""

    __slots__ = [
        '_header',
        '_trial_start',
        '_timeout_time',
        '_trial_name',
        '_trial_number',
        '_num_actors',
        '_robot_start',
        '_robot_goal',
        '_dist_to_target',
        '_min_dist_to_target',
        '_robot_poses',
        '_robot_poses_ts',
        '_min_dist_to_ped',
        '_robot_on_person_intimate_dist_violations',
        '_person_on_robot_intimate_dist_violations',
        '_robot_on_person_personal_dist_violations',
        '_person_on_robot_personal_dist_violations',
        '_robot_on_person_collisions',
        '_person_on_robot_collisions',
        '_obj_collisions',
        '_path_length',
        '_path_irregularity',
        '_time_not_moving',
        '_time_in_personal_space',
        '_minimum_time_to_collision',
        '_movement_jerk',
    ]

    _fields_and_field_types = {
        'header': 'std_msgs/Header',
        'trial_start': 'builtin_interfaces/Time',
        'timeout_time': 'double',
        'trial_name': 'string',
        'trial_number': 'uint16',
        'num_actors': 'uint32',
        'robot_start': 'geometry_msgs/Pose',
        'robot_goal': 'geometry_msgs/Pose',
        'dist_to_target': 'double',
        'min_dist_to_target': 'double',
        'robot_poses': 'sequence<geometry_msgs/Pose>',
        'robot_poses_ts': 'sequence<builtin_interfaces/Time>',
        'min_dist_to_ped': 'double',
        'robot_on_person_intimate_dist_violations': 'uint32',
        'person_on_robot_intimate_dist_violations': 'uint32',
        'robot_on_person_personal_dist_violations': 'uint32',
        'person_on_robot_personal_dist_violations': 'uint32',
        'robot_on_person_collisions': 'uint32',
        'person_on_robot_collisions': 'uint32',
        'obj_collisions': 'uint32',
        'path_length': 'double',
        'path_irregularity': 'double',
        'time_not_moving': 'double',
        'time_in_personal_space': 'double',
        'minimum_time_to_collision': 'double',
        'movement_jerk': 'double',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.NamespacedType(['std_msgs', 'msg'], 'Header'),  # noqa: E501
        rosidl_parser.definition.NamespacedType(['builtin_interfaces', 'msg'], 'Time'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.UnboundedString(),  # noqa: E501
        rosidl_parser.definition.BasicType('uint16'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint32'),  # noqa: E501
        rosidl_parser.definition.NamespacedType(['geometry_msgs', 'msg'], 'Pose'),  # noqa: E501
        rosidl_parser.definition.NamespacedType(['geometry_msgs', 'msg'], 'Pose'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.UnboundedSequence(rosidl_parser.definition.NamespacedType(['geometry_msgs', 'msg'], 'Pose')),  # noqa: E501
        rosidl_parser.definition.UnboundedSequence(rosidl_parser.definition.NamespacedType(['builtin_interfaces', 'msg'], 'Time')),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint32'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint32'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint32'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint32'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint32'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint32'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint32'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        from std_msgs.msg import Header
        self.header = kwargs.get('header', Header())
        from builtin_interfaces.msg import Time
        self.trial_start = kwargs.get('trial_start', Time())
        self.timeout_time = kwargs.get('timeout_time', float())
        self.trial_name = kwargs.get('trial_name', str())
        self.trial_number = kwargs.get('trial_number', int())
        self.num_actors = kwargs.get('num_actors', int())
        from geometry_msgs.msg import Pose
        self.robot_start = kwargs.get('robot_start', Pose())
        from geometry_msgs.msg import Pose
        self.robot_goal = kwargs.get('robot_goal', Pose())
        self.dist_to_target = kwargs.get('dist_to_target', float())
        self.min_dist_to_target = kwargs.get('min_dist_to_target', float())
        self.robot_poses = kwargs.get('robot_poses', [])
        self.robot_poses_ts = kwargs.get('robot_poses_ts', [])
        self.min_dist_to_ped = kwargs.get('min_dist_to_ped', float())
        self.robot_on_person_intimate_dist_violations = kwargs.get('robot_on_person_intimate_dist_violations', int())
        self.person_on_robot_intimate_dist_violations = kwargs.get('person_on_robot_intimate_dist_violations', int())
        self.robot_on_person_personal_dist_violations = kwargs.get('robot_on_person_personal_dist_violations', int())
        self.person_on_robot_personal_dist_violations = kwargs.get('person_on_robot_personal_dist_violations', int())
        self.robot_on_person_collisions = kwargs.get('robot_on_person_collisions', int())
        self.person_on_robot_collisions = kwargs.get('person_on_robot_collisions', int())
        self.obj_collisions = kwargs.get('obj_collisions', int())
        self.path_length = kwargs.get('path_length', float())
        self.path_irregularity = kwargs.get('path_irregularity', float())
        self.time_not_moving = kwargs.get('time_not_moving', float())
        self.time_in_personal_space = kwargs.get('time_in_personal_space', float())
        self.minimum_time_to_collision = kwargs.get('minimum_time_to_collision', float())
        self.movement_jerk = kwargs.get('movement_jerk', float())

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
        if self.trial_start != other.trial_start:
            return False
        if self.timeout_time != other.timeout_time:
            return False
        if self.trial_name != other.trial_name:
            return False
        if self.trial_number != other.trial_number:
            return False
        if self.num_actors != other.num_actors:
            return False
        if self.robot_start != other.robot_start:
            return False
        if self.robot_goal != other.robot_goal:
            return False
        if self.dist_to_target != other.dist_to_target:
            return False
        if self.min_dist_to_target != other.min_dist_to_target:
            return False
        if self.robot_poses != other.robot_poses:
            return False
        if self.robot_poses_ts != other.robot_poses_ts:
            return False
        if self.min_dist_to_ped != other.min_dist_to_ped:
            return False
        if self.robot_on_person_intimate_dist_violations != other.robot_on_person_intimate_dist_violations:
            return False
        if self.person_on_robot_intimate_dist_violations != other.person_on_robot_intimate_dist_violations:
            return False
        if self.robot_on_person_personal_dist_violations != other.robot_on_person_personal_dist_violations:
            return False
        if self.person_on_robot_personal_dist_violations != other.person_on_robot_personal_dist_violations:
            return False
        if self.robot_on_person_collisions != other.robot_on_person_collisions:
            return False
        if self.person_on_robot_collisions != other.person_on_robot_collisions:
            return False
        if self.obj_collisions != other.obj_collisions:
            return False
        if self.path_length != other.path_length:
            return False
        if self.path_irregularity != other.path_irregularity:
            return False
        if self.time_not_moving != other.time_not_moving:
            return False
        if self.time_in_personal_space != other.time_in_personal_space:
            return False
        if self.minimum_time_to_collision != other.minimum_time_to_collision:
            return False
        if self.movement_jerk != other.movement_jerk:
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
    def trial_start(self):
        """Message field 'trial_start'."""
        return self._trial_start

    @trial_start.setter
    def trial_start(self, value):
        if __debug__:
            from builtin_interfaces.msg import Time
            assert \
                isinstance(value, Time), \
                "The 'trial_start' field must be a sub message of type 'Time'"
        self._trial_start = value

    @builtins.property
    def timeout_time(self):
        """Message field 'timeout_time'."""
        return self._timeout_time

    @timeout_time.setter
    def timeout_time(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'timeout_time' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'timeout_time' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._timeout_time = value

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
    def num_actors(self):
        """Message field 'num_actors'."""
        return self._num_actors

    @num_actors.setter
    def num_actors(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'num_actors' field must be of type 'int'"
            assert value >= 0 and value < 4294967296, \
                "The 'num_actors' field must be an unsigned integer in [0, 4294967295]"
        self._num_actors = value

    @builtins.property
    def robot_start(self):
        """Message field 'robot_start'."""
        return self._robot_start

    @robot_start.setter
    def robot_start(self, value):
        if __debug__:
            from geometry_msgs.msg import Pose
            assert \
                isinstance(value, Pose), \
                "The 'robot_start' field must be a sub message of type 'Pose'"
        self._robot_start = value

    @builtins.property
    def robot_goal(self):
        """Message field 'robot_goal'."""
        return self._robot_goal

    @robot_goal.setter
    def robot_goal(self, value):
        if __debug__:
            from geometry_msgs.msg import Pose
            assert \
                isinstance(value, Pose), \
                "The 'robot_goal' field must be a sub message of type 'Pose'"
        self._robot_goal = value

    @builtins.property
    def dist_to_target(self):
        """Message field 'dist_to_target'."""
        return self._dist_to_target

    @dist_to_target.setter
    def dist_to_target(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'dist_to_target' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'dist_to_target' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._dist_to_target = value

    @builtins.property
    def min_dist_to_target(self):
        """Message field 'min_dist_to_target'."""
        return self._min_dist_to_target

    @min_dist_to_target.setter
    def min_dist_to_target(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'min_dist_to_target' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'min_dist_to_target' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._min_dist_to_target = value

    @builtins.property
    def robot_poses(self):
        """Message field 'robot_poses'."""
        return self._robot_poses

    @robot_poses.setter
    def robot_poses(self, value):
        if __debug__:
            from geometry_msgs.msg import Pose
            from collections.abc import Sequence
            from collections.abc import Set
            from collections import UserList
            from collections import UserString
            assert \
                ((isinstance(value, Sequence) or
                  isinstance(value, Set) or
                  isinstance(value, UserList)) and
                 not isinstance(value, str) and
                 not isinstance(value, UserString) and
                 all(isinstance(v, Pose) for v in value) and
                 True), \
                "The 'robot_poses' field must be a set or sequence and each value of type 'Pose'"
        self._robot_poses = value

    @builtins.property
    def robot_poses_ts(self):
        """Message field 'robot_poses_ts'."""
        return self._robot_poses_ts

    @robot_poses_ts.setter
    def robot_poses_ts(self, value):
        if __debug__:
            from builtin_interfaces.msg import Time
            from collections.abc import Sequence
            from collections.abc import Set
            from collections import UserList
            from collections import UserString
            assert \
                ((isinstance(value, Sequence) or
                  isinstance(value, Set) or
                  isinstance(value, UserList)) and
                 not isinstance(value, str) and
                 not isinstance(value, UserString) and
                 all(isinstance(v, Time) for v in value) and
                 True), \
                "The 'robot_poses_ts' field must be a set or sequence and each value of type 'Time'"
        self._robot_poses_ts = value

    @builtins.property
    def min_dist_to_ped(self):
        """Message field 'min_dist_to_ped'."""
        return self._min_dist_to_ped

    @min_dist_to_ped.setter
    def min_dist_to_ped(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'min_dist_to_ped' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'min_dist_to_ped' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._min_dist_to_ped = value

    @builtins.property
    def robot_on_person_intimate_dist_violations(self):
        """Message field 'robot_on_person_intimate_dist_violations'."""
        return self._robot_on_person_intimate_dist_violations

    @robot_on_person_intimate_dist_violations.setter
    def robot_on_person_intimate_dist_violations(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'robot_on_person_intimate_dist_violations' field must be of type 'int'"
            assert value >= 0 and value < 4294967296, \
                "The 'robot_on_person_intimate_dist_violations' field must be an unsigned integer in [0, 4294967295]"
        self._robot_on_person_intimate_dist_violations = value

    @builtins.property
    def person_on_robot_intimate_dist_violations(self):
        """Message field 'person_on_robot_intimate_dist_violations'."""
        return self._person_on_robot_intimate_dist_violations

    @person_on_robot_intimate_dist_violations.setter
    def person_on_robot_intimate_dist_violations(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'person_on_robot_intimate_dist_violations' field must be of type 'int'"
            assert value >= 0 and value < 4294967296, \
                "The 'person_on_robot_intimate_dist_violations' field must be an unsigned integer in [0, 4294967295]"
        self._person_on_robot_intimate_dist_violations = value

    @builtins.property
    def robot_on_person_personal_dist_violations(self):
        """Message field 'robot_on_person_personal_dist_violations'."""
        return self._robot_on_person_personal_dist_violations

    @robot_on_person_personal_dist_violations.setter
    def robot_on_person_personal_dist_violations(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'robot_on_person_personal_dist_violations' field must be of type 'int'"
            assert value >= 0 and value < 4294967296, \
                "The 'robot_on_person_personal_dist_violations' field must be an unsigned integer in [0, 4294967295]"
        self._robot_on_person_personal_dist_violations = value

    @builtins.property
    def person_on_robot_personal_dist_violations(self):
        """Message field 'person_on_robot_personal_dist_violations'."""
        return self._person_on_robot_personal_dist_violations

    @person_on_robot_personal_dist_violations.setter
    def person_on_robot_personal_dist_violations(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'person_on_robot_personal_dist_violations' field must be of type 'int'"
            assert value >= 0 and value < 4294967296, \
                "The 'person_on_robot_personal_dist_violations' field must be an unsigned integer in [0, 4294967295]"
        self._person_on_robot_personal_dist_violations = value

    @builtins.property
    def robot_on_person_collisions(self):
        """Message field 'robot_on_person_collisions'."""
        return self._robot_on_person_collisions

    @robot_on_person_collisions.setter
    def robot_on_person_collisions(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'robot_on_person_collisions' field must be of type 'int'"
            assert value >= 0 and value < 4294967296, \
                "The 'robot_on_person_collisions' field must be an unsigned integer in [0, 4294967295]"
        self._robot_on_person_collisions = value

    @builtins.property
    def person_on_robot_collisions(self):
        """Message field 'person_on_robot_collisions'."""
        return self._person_on_robot_collisions

    @person_on_robot_collisions.setter
    def person_on_robot_collisions(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'person_on_robot_collisions' field must be of type 'int'"
            assert value >= 0 and value < 4294967296, \
                "The 'person_on_robot_collisions' field must be an unsigned integer in [0, 4294967295]"
        self._person_on_robot_collisions = value

    @builtins.property
    def obj_collisions(self):
        """Message field 'obj_collisions'."""
        return self._obj_collisions

    @obj_collisions.setter
    def obj_collisions(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'obj_collisions' field must be of type 'int'"
            assert value >= 0 and value < 4294967296, \
                "The 'obj_collisions' field must be an unsigned integer in [0, 4294967295]"
        self._obj_collisions = value

    @builtins.property
    def path_length(self):
        """Message field 'path_length'."""
        return self._path_length

    @path_length.setter
    def path_length(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'path_length' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'path_length' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._path_length = value

    @builtins.property
    def path_irregularity(self):
        """Message field 'path_irregularity'."""
        return self._path_irregularity

    @path_irregularity.setter
    def path_irregularity(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'path_irregularity' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'path_irregularity' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._path_irregularity = value

    @builtins.property
    def time_not_moving(self):
        """Message field 'time_not_moving'."""
        return self._time_not_moving

    @time_not_moving.setter
    def time_not_moving(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'time_not_moving' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'time_not_moving' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._time_not_moving = value

    @builtins.property
    def time_in_personal_space(self):
        """Message field 'time_in_personal_space'."""
        return self._time_in_personal_space

    @time_in_personal_space.setter
    def time_in_personal_space(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'time_in_personal_space' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'time_in_personal_space' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._time_in_personal_space = value

    @builtins.property
    def minimum_time_to_collision(self):
        """Message field 'minimum_time_to_collision'."""
        return self._minimum_time_to_collision

    @minimum_time_to_collision.setter
    def minimum_time_to_collision(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'minimum_time_to_collision' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'minimum_time_to_collision' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._minimum_time_to_collision = value

    @builtins.property
    def movement_jerk(self):
        """Message field 'movement_jerk'."""
        return self._movement_jerk

    @movement_jerk.setter
    def movement_jerk(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'movement_jerk' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'movement_jerk' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._movement_jerk = value
