# Generated by the protocol buffer compiler.  DO NOT EDIT!
# source: message/motion/GetupCommand.proto

import sys
_b=sys.version_info[0]<3 and (lambda x:x) or (lambda x:x.encode('latin1'))
from google.protobuf import descriptor as _descriptor
from google.protobuf import message as _message
from google.protobuf import reflection as _reflection
from google.protobuf import symbol_database as _symbol_database
# @@protoc_insertion_point(imports)

_sym_db = _symbol_database.Default()




DESCRIPTOR = _descriptor.FileDescriptor(
  name='message/motion/GetupCommand.proto',
  package='message.motion',
  syntax='proto3',
  serialized_options=None,
  serialized_pb=_b('\n!message/motion/GetupCommand.proto\x12\x0emessage.motion\"\x0e\n\x0c\x45xecuteGetup\"\x0b\n\tKillGetupb\x06proto3')
)




_EXECUTEGETUP = _descriptor.Descriptor(
  name='ExecuteGetup',
  full_name='message.motion.ExecuteGetup',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
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
  serialized_start=53,
  serialized_end=67,
)


_KILLGETUP = _descriptor.Descriptor(
  name='KillGetup',
  full_name='message.motion.KillGetup',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
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
  serialized_start=69,
  serialized_end=80,
)

DESCRIPTOR.message_types_by_name['ExecuteGetup'] = _EXECUTEGETUP
DESCRIPTOR.message_types_by_name['KillGetup'] = _KILLGETUP
_sym_db.RegisterFileDescriptor(DESCRIPTOR)

ExecuteGetup = _reflection.GeneratedProtocolMessageType('ExecuteGetup', (_message.Message,), dict(
  DESCRIPTOR = _EXECUTEGETUP,
  __module__ = 'message.motion.GetupCommand_pb2'
  # @@protoc_insertion_point(class_scope:message.motion.ExecuteGetup)
  ))
_sym_db.RegisterMessage(ExecuteGetup)

KillGetup = _reflection.GeneratedProtocolMessageType('KillGetup', (_message.Message,), dict(
  DESCRIPTOR = _KILLGETUP,
  __module__ = 'message.motion.GetupCommand_pb2'
  # @@protoc_insertion_point(class_scope:message.motion.KillGetup)
  ))
_sym_db.RegisterMessage(KillGetup)


# @@protoc_insertion_point(module_scope)