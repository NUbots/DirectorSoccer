# Generated by the protocol buffer compiler.  DO NOT EDIT!
# source: message/platform/gazebo/Ball.proto

import sys
_b=sys.version_info[0]<3 and (lambda x:x) or (lambda x:x.encode('latin1'))
from google.protobuf import descriptor as _descriptor
from google.protobuf import message as _message
from google.protobuf import reflection as _reflection
from google.protobuf import symbol_database as _symbol_database
# @@protoc_insertion_point(imports)

_sym_db = _symbol_database.Default()


import Vector_pb2 as Vector__pb2


DESCRIPTOR = _descriptor.FileDescriptor(
  name='message/platform/gazebo/Ball.proto',
  package='message.platform.gazebo',
  syntax='proto3',
  serialized_options=None,
  serialized_pb=_b('\n\"message/platform/gazebo/Ball.proto\x12\x17message.platform.gazebo\x1a\x0cVector.proto\"/\n\x04\x42\x61ll\x12\x13\n\x04rBWw\x18\x01 \x01(\x0b\x32\x05.vec3\x12\x12\n\x03vBw\x18\x02 \x01(\x0b\x32\x05.vec3b\x06proto3')
  ,
  dependencies=[Vector__pb2.DESCRIPTOR,])




_BALL = _descriptor.Descriptor(
  name='Ball',
  full_name='message.platform.gazebo.Ball',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='rBWw', full_name='message.platform.gazebo.Ball.rBWw', index=0,
      number=1, type=11, cpp_type=10, label=1,
      has_default_value=False, default_value=None,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='vBw', full_name='message.platform.gazebo.Ball.vBw', index=1,
      number=2, type=11, cpp_type=10, label=1,
      has_default_value=False, default_value=None,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
  ],
  extensions=[
  ],
  nested_types=[],
  enum_types=[
  ],
  serialized_options=None,
  is_extendable=False,
  syntax='proto3',
  extension_ranges=[],
  oneofs=[
  ],
  serialized_start=77,
  serialized_end=124,
)

_BALL.fields_by_name['rBWw'].message_type = Vector__pb2._VEC3
_BALL.fields_by_name['vBw'].message_type = Vector__pb2._VEC3
DESCRIPTOR.message_types_by_name['Ball'] = _BALL
_sym_db.RegisterFileDescriptor(DESCRIPTOR)

Ball = _reflection.GeneratedProtocolMessageType('Ball', (_message.Message,), dict(
  DESCRIPTOR = _BALL,
  __module__ = 'message.platform.gazebo.Ball_pb2'
  # @@protoc_insertion_point(class_scope:message.platform.gazebo.Ball)
  ))
_sym_db.RegisterMessage(Ball)


# @@protoc_insertion_point(module_scope)