# Generated by the protocol buffer compiler.  DO NOT EDIT!
# source: message/behaviour/Look.proto

import sys
_b=sys.version_info[0]<3 and (lambda x:x) or (lambda x:x.encode('latin1'))
from google.protobuf import descriptor as _descriptor
from google.protobuf import message as _message
from google.protobuf import reflection as _reflection
from google.protobuf import symbol_database as _symbol_database
# @@protoc_insertion_point(imports)

_sym_db = _symbol_database.Default()


from google.protobuf import duration_pb2 as google_dot_protobuf_dot_duration__pb2
import Vector_pb2 as Vector__pb2


DESCRIPTOR = _descriptor.FileDescriptor(
  name='message/behaviour/Look.proto',
  package='message.behaviour',
  syntax='proto3',
  serialized_options=None,
  serialized_pb=_b('\n\x1cmessage/behaviour/Look.proto\x12\x11message.behaviour\x1a\x1egoogle/protobuf/duration.proto\x1a\x0cVector.proto\"\x8d\x02\n\x04Look\x1a\x38\n\x08\x46ixation\x12\x14\n\x05\x61ngle\x18\x01 \x01(\x0b\x32\x05.vec2\x12\x16\n\x07\x61rcSize\x18\x02 \x01(\x0b\x32\x05.vec2\x1a\x65\n\x07Saccade\x12,\n\tdwellTime\x18\x01 \x01(\x0b\x32\x19.google.protobuf.Duration\x12\x14\n\x05\x61ngle\x18\x02 \x01(\x0b\x32\x05.vec2\x12\x16\n\x07\x61rcSize\x18\x03 \x01(\x0b\x32\x05.vec2\x1a\x33\n\x03Pan\x12\x14\n\x05\x61ngle\x18\x01 \x01(\x0b\x32\x05.vec2\x12\x16\n\x07\x61rcSize\x18\x02 \x01(\x0b\x32\x05.vec2\x1a/\n\x0cPanSelection\x12\x1f\n\x17lookAtGoalInsteadOfBall\x18\x01 \x01(\x08\x62\x06proto3')
  ,
  dependencies=[google_dot_protobuf_dot_duration__pb2.DESCRIPTOR,Vector__pb2.DESCRIPTOR,])




_LOOK_FIXATION = _descriptor.Descriptor(
  name='Fixation',
  full_name='message.behaviour.Look.Fixation',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='angle', full_name='message.behaviour.Look.Fixation.angle', index=0,
      number=1, type=11, cpp_type=10, label=1,
      has_default_value=False, default_value=None,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='arcSize', full_name='message.behaviour.Look.Fixation.arcSize', index=1,
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
  serialized_start=106,
  serialized_end=162,
)

_LOOK_SACCADE = _descriptor.Descriptor(
  name='Saccade',
  full_name='message.behaviour.Look.Saccade',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='dwellTime', full_name='message.behaviour.Look.Saccade.dwellTime', index=0,
      number=1, type=11, cpp_type=10, label=1,
      has_default_value=False, default_value=None,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='angle', full_name='message.behaviour.Look.Saccade.angle', index=1,
      number=2, type=11, cpp_type=10, label=1,
      has_default_value=False, default_value=None,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='arcSize', full_name='message.behaviour.Look.Saccade.arcSize', index=2,
      number=3, type=11, cpp_type=10, label=1,
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
  serialized_start=164,
  serialized_end=265,
)

_LOOK_PAN = _descriptor.Descriptor(
  name='Pan',
  full_name='message.behaviour.Look.Pan',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='angle', full_name='message.behaviour.Look.Pan.angle', index=0,
      number=1, type=11, cpp_type=10, label=1,
      has_default_value=False, default_value=None,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='arcSize', full_name='message.behaviour.Look.Pan.arcSize', index=1,
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
  serialized_start=267,
  serialized_end=318,
)

_LOOK_PANSELECTION = _descriptor.Descriptor(
  name='PanSelection',
  full_name='message.behaviour.Look.PanSelection',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='lookAtGoalInsteadOfBall', full_name='message.behaviour.Look.PanSelection.lookAtGoalInsteadOfBall', index=0,
      number=1, type=8, cpp_type=7, label=1,
      has_default_value=False, default_value=False,
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
  serialized_start=320,
  serialized_end=367,
)

_LOOK = _descriptor.Descriptor(
  name='Look',
  full_name='message.behaviour.Look',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
  ],
  extensions=[
  ],
  nested_types=[_LOOK_FIXATION, _LOOK_SACCADE, _LOOK_PAN, _LOOK_PANSELECTION, ],
  enum_types=[
  ],
  serialized_options=None,
  is_extendable=False,
  syntax='proto3',
  extension_ranges=[],
  oneofs=[
  ],
  serialized_start=98,
  serialized_end=367,
)

_LOOK_FIXATION.fields_by_name['angle'].message_type = Vector__pb2._VEC2
_LOOK_FIXATION.fields_by_name['arcSize'].message_type = Vector__pb2._VEC2
_LOOK_FIXATION.containing_type = _LOOK
_LOOK_SACCADE.fields_by_name['dwellTime'].message_type = google_dot_protobuf_dot_duration__pb2._DURATION
_LOOK_SACCADE.fields_by_name['angle'].message_type = Vector__pb2._VEC2
_LOOK_SACCADE.fields_by_name['arcSize'].message_type = Vector__pb2._VEC2
_LOOK_SACCADE.containing_type = _LOOK
_LOOK_PAN.fields_by_name['angle'].message_type = Vector__pb2._VEC2
_LOOK_PAN.fields_by_name['arcSize'].message_type = Vector__pb2._VEC2
_LOOK_PAN.containing_type = _LOOK
_LOOK_PANSELECTION.containing_type = _LOOK
DESCRIPTOR.message_types_by_name['Look'] = _LOOK
_sym_db.RegisterFileDescriptor(DESCRIPTOR)

Look = _reflection.GeneratedProtocolMessageType('Look', (_message.Message,), dict(

  Fixation = _reflection.GeneratedProtocolMessageType('Fixation', (_message.Message,), dict(
    DESCRIPTOR = _LOOK_FIXATION,
    __module__ = 'message.behaviour.Look_pb2'
    # @@protoc_insertion_point(class_scope:message.behaviour.Look.Fixation)
    ))
  ,

  Saccade = _reflection.GeneratedProtocolMessageType('Saccade', (_message.Message,), dict(
    DESCRIPTOR = _LOOK_SACCADE,
    __module__ = 'message.behaviour.Look_pb2'
    # @@protoc_insertion_point(class_scope:message.behaviour.Look.Saccade)
    ))
  ,

  Pan = _reflection.GeneratedProtocolMessageType('Pan', (_message.Message,), dict(
    DESCRIPTOR = _LOOK_PAN,
    __module__ = 'message.behaviour.Look_pb2'
    # @@protoc_insertion_point(class_scope:message.behaviour.Look.Pan)
    ))
  ,

  PanSelection = _reflection.GeneratedProtocolMessageType('PanSelection', (_message.Message,), dict(
    DESCRIPTOR = _LOOK_PANSELECTION,
    __module__ = 'message.behaviour.Look_pb2'
    # @@protoc_insertion_point(class_scope:message.behaviour.Look.PanSelection)
    ))
  ,
  DESCRIPTOR = _LOOK,
  __module__ = 'message.behaviour.Look_pb2'
  # @@protoc_insertion_point(class_scope:message.behaviour.Look)
  ))
_sym_db.RegisterMessage(Look)
_sym_db.RegisterMessage(Look.Fixation)
_sym_db.RegisterMessage(Look.Saccade)
_sym_db.RegisterMessage(Look.Pan)
_sym_db.RegisterMessage(Look.PanSelection)


# @@protoc_insertion_point(module_scope)