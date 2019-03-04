# This Python file uses the following encoding: utf-8
"""autogenerated by genpy from custom_msg_topic/custom_msg.msg. Do not edit."""
import sys
python3 = True if sys.hexversion > 0x03000000 else False
import genpy
import struct


class custom_msg(genpy.Message):
  _md5sum = "56c5740684da4cec8bf62501dd7b9504"
  _type = "custom_msg_topic/custom_msg"
  _has_header = False #flag to mark the presence of a Header object
  _full_text = """float32 levels_of_anger
float32 min_lidar_data
float32[] Avoidance_Classification_group
float32 Speed_change_of_obstacle 
"""
  __slots__ = ['levels_of_anger','min_lidar_data','Avoidance_Classification_group','Speed_change_of_obstacle']
  _slot_types = ['float32','float32','float32[]','float32']

  def __init__(self, *args, **kwds):
    """
    Constructor. Any message fields that are implicitly/explicitly
    set to None will be assigned a default value. The recommend
    use is keyword arguments as this is more robust to future message
    changes.  You cannot mix in-order arguments and keyword arguments.

    The available fields are:
       levels_of_anger,min_lidar_data,Avoidance_Classification_group,Speed_change_of_obstacle

    :param args: complete set of field values, in .msg order
    :param kwds: use keyword arguments corresponding to message field names
    to set specific fields.
    """
    if args or kwds:
      super(custom_msg, self).__init__(*args, **kwds)
      #message fields cannot be None, assign default values for those that are
      if self.levels_of_anger is None:
        self.levels_of_anger = 0.
      if self.min_lidar_data is None:
        self.min_lidar_data = 0.
      if self.Avoidance_Classification_group is None:
        self.Avoidance_Classification_group = []
      if self.Speed_change_of_obstacle is None:
        self.Speed_change_of_obstacle = 0.
    else:
      self.levels_of_anger = 0.
      self.min_lidar_data = 0.
      self.Avoidance_Classification_group = []
      self.Speed_change_of_obstacle = 0.

  def _get_types(self):
    """
    internal API method
    """
    return self._slot_types

  def serialize(self, buff):
    """
    serialize message into buffer
    :param buff: buffer, ``StringIO``
    """
    try:
      _x = self
      buff.write(_get_struct_2f().pack(_x.levels_of_anger, _x.min_lidar_data))
      length = len(self.Avoidance_Classification_group)
      buff.write(_struct_I.pack(length))
      pattern = '<%sf'%length
      buff.write(struct.pack(pattern, *self.Avoidance_Classification_group))
      buff.write(_get_struct_f().pack(self.Speed_change_of_obstacle))
    except struct.error as se: self._check_types(struct.error("%s: '%s' when writing '%s'" % (type(se), str(se), str(locals().get('_x', self)))))
    except TypeError as te: self._check_types(ValueError("%s: '%s' when writing '%s'" % (type(te), str(te), str(locals().get('_x', self)))))

  def deserialize(self, str):
    """
    unpack serialized message in str into this message instance
    :param str: byte array of serialized message, ``str``
    """
    try:
      end = 0
      _x = self
      start = end
      end += 8
      (_x.levels_of_anger, _x.min_lidar_data,) = _get_struct_2f().unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      pattern = '<%sf'%length
      start = end
      end += struct.calcsize(pattern)
      self.Avoidance_Classification_group = struct.unpack(pattern, str[start:end])
      start = end
      end += 4
      (self.Speed_change_of_obstacle,) = _get_struct_f().unpack(str[start:end])
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e) #most likely buffer underfill


  def serialize_numpy(self, buff, numpy):
    """
    serialize message with numpy array types into buffer
    :param buff: buffer, ``StringIO``
    :param numpy: numpy python module
    """
    try:
      _x = self
      buff.write(_get_struct_2f().pack(_x.levels_of_anger, _x.min_lidar_data))
      length = len(self.Avoidance_Classification_group)
      buff.write(_struct_I.pack(length))
      pattern = '<%sf'%length
      buff.write(self.Avoidance_Classification_group.tostring())
      buff.write(_get_struct_f().pack(self.Speed_change_of_obstacle))
    except struct.error as se: self._check_types(struct.error("%s: '%s' when writing '%s'" % (type(se), str(se), str(locals().get('_x', self)))))
    except TypeError as te: self._check_types(ValueError("%s: '%s' when writing '%s'" % (type(te), str(te), str(locals().get('_x', self)))))

  def deserialize_numpy(self, str, numpy):
    """
    unpack serialized message in str into this message instance using numpy for array types
    :param str: byte array of serialized message, ``str``
    :param numpy: numpy python module
    """
    try:
      end = 0
      _x = self
      start = end
      end += 8
      (_x.levels_of_anger, _x.min_lidar_data,) = _get_struct_2f().unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      pattern = '<%sf'%length
      start = end
      end += struct.calcsize(pattern)
      self.Avoidance_Classification_group = numpy.frombuffer(str[start:end], dtype=numpy.float32, count=length)
      start = end
      end += 4
      (self.Speed_change_of_obstacle,) = _get_struct_f().unpack(str[start:end])
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e) #most likely buffer underfill

_struct_I = genpy.struct_I
def _get_struct_I():
    global _struct_I
    return _struct_I
_struct_2f = None
def _get_struct_2f():
    global _struct_2f
    if _struct_2f is None:
        _struct_2f = struct.Struct("<2f")
    return _struct_2f
_struct_f = None
def _get_struct_f():
    global _struct_f
    if _struct_f is None:
        _struct_f = struct.Struct("<f")
    return _struct_f
