# Generated by the protocol buffer compiler.  DO NOT EDIT!
# source: message/support/nusight/Command.proto

import sys
_b=sys.version_info[0]<3 and (lambda x:x) or (lambda x:x.encode('latin1'))
from google.protobuf import descriptor as _descriptor
from google.protobuf import message as _message
from google.protobuf import reflection as _reflection
from google.protobuf import symbol_database as _symbol_database
# @@protoc_insertion_point(imports)

_sym_db = _symbol_database.Default()




DESCRIPTOR = _descriptor.FileDescriptor(
  name='message/support/nusight/Command.proto',
  package='message.support.nusight',
  syntax='proto3',
  serialized_options=None,
  serialized_pb=_b('\n%message/support/nusight/Command.proto\x12\x17message.support.nusight\"\x1a\n\x07\x43ommand\x12\x0f\n\x07\x63ommand\x18\x01 \x01(\tb\x06proto3')
)




_COMMAND = _descriptor.Descriptor(
  name='Command',
  full_name='message.support.nusight.Command',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='command', full_name='message.support.nusight.Command.command', index=0,
      number=1, type=9, cpp_type=9, label=1,
      has_default_value=False, default_value=_b("").decode('utf-8'),
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
  serialized_start=66,
  serialized_end=92,
)

DESCRIPTOR.message_types_by_name['Command'] = _COMMAND
_sym_db.RegisterFileDescriptor(DESCRIPTOR)

Command = _reflection.GeneratedProtocolMessageType('Command', (_message.Message,), dict(
  DESCRIPTOR = _COMMAND,
  __module__ = 'message.support.nusight.Command_pb2'
  # @@protoc_insertion_point(class_scope:message.support.nusight.Command)
  ))
_sym_db.RegisterMessage(Command)


# @@protoc_insertion_point(module_scope)