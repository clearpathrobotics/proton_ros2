import inspect
import importlib
import yaml

from rosidl_runtime_py.utilities import get_message
from rosidl_parser.definition import (
    BasicType,
    NamespacedType,
    UnboundedSequence,
    BoundedSequence,
    UnboundedString,
    BoundedString,
    Array
)

from proton_ros2.message_config import ProtonROS2Config

from builtin_interfaces.msg import Time

def ros_type_to_proton_type(field_type):
    """Map a ROS2 field type to a Proton type."""
    # Scalars
    if isinstance(field_type, BasicType):
        mapping = {
            'double': 'double',
            'float': 'float',
            'int8': 'int32',
            'int16': 'int32',
            'int32': 'int32',
            'int64': 'int64',
            'uint8': 'uint32',
            'uint16': 'uint32',
            'uint32': 'uint32',
            'uint64': 'uint64',
            'boolean': 'bool',
            'byte': 'uint32',
            'char': 'uint32',
            'string': 'string',
            'octet': 'int32'
        }
        return mapping.get(field_type.typename, 'unknown')

    # Strings
    if isinstance(field_type, (UnboundedString, BoundedString)):
        return 'string'

    # Sequences or arrays
    if isinstance(field_type, (UnboundedSequence, BoundedSequence, Array)):
        elem_type = field_type.value_type
        if isinstance(elem_type, BasicType):
            mapping = {
                'double': 'list_double',
                'float': 'list_float',
                'int8': 'list_int32',
                'int16': 'list_int32',
                'int32': 'list_int32',
                'int64': 'list_int64',
                'uint8': 'bytes',
                'uint16': 'list_uint32',
                'uint32': 'list_uint32',
                'uint64': 'list_uint64',
                'boolean': 'list_bool',
                'string': 'list_string',
                'byte': 'bytes',
                'octet': 'list_int32'
            }
            return mapping.get(elem_type.typename, 'list_unknown_map')
        elif isinstance(elem_type, (UnboundedString, BoundedString)):
            return 'list_string'
        elif isinstance(elem_type, NamespacedType):
            if isinstance(field_type, UnboundedSequence):
                return 'list_unbounded'
            elif isinstance(field_type, BoundedSequence):
                return 'list_bounded'
            elif isinstance(field_type, Array):
                return 'list_array'
        else:
            print(f'Field type {field_type} elem_type {elem_type}')
            return 'list_unknown'

    # Nested messages
    if isinstance(field_type, NamespacedType):
        return 'nested'

    return 'unknown_type'


def flatten_message(msg_type, ros_path_prefix="", proton_prefix=""):
    """Recursively flatten a ROS2 message into {field_name: proton_type}."""
    fields = {}
    for slot, type_ in zip(msg_type.__slots__, msg_type.SLOT_TYPES):
        name = slot.lstrip('_')
        ros_path = f"{ros_path_prefix}{name}"
        proton_name = f"{proton_prefix}{name}"
        proton_type = ros_type_to_proton_type(type_)
        if proton_type == "unknown":
            raise KeyError(f"UNKNOWN TYPE {type_.typename}")

        # Nested message
        if proton_type == 'nested':
            nested_cls = get_message("/".join(type_.namespaces + [type_.name]))
            # Define path to stamp
            if name == 'stamp' and nested_cls == Time:
                fields.update({proton_name: {'stamp': ros_path}})
            else:
                fields.update(flatten_message(nested_cls, ros_path_prefix=f"{ros_path}.", proton_prefix=f"{proton_name}_"))

        # Sequence of nested messages
        elif proton_type == 'list_unbounded' or proton_type == 'list_bounded':
            nested_cls = get_message("/".join(type_.value_type.namespaces + [type_.value_type.name]))
            # Flatten each subfield with array-style prefix
            nested_fields = flatten_message(nested_cls, ros_path_prefix=f"{ros_path}.", proton_prefix=f"{proton_name}_")
            for k, v in nested_fields.items():
                v = v.copy()
                v['subpath'] = v['ros_path'].split('.')[1]
                v['ros_path'] = ros_path
                v['array'] = True
                v['bounded'] = getattr(type_, 'maximum', 0)
                if v['proton_type'] != "bytes" and not v['proton_type'].startswith('list_'):
                    v['proton_type'] = f'list_{v['proton_type']}'
                fields[k] = v
        # Array of nested messages
        elif proton_type == 'list_array':
            nested_cls = get_message("/".join(type_.value_type.namespaces + [type_.value_type.name]))
            # Flatten each subfield with array-style prefix
            nested_fields = flatten_message(nested_cls, ros_path_prefix=f"{ros_path}.", proton_prefix=f"{proton_name}_")
            for k, v in nested_fields.items():
                v = v.copy()
                v['subpath'] = v['ros_path'].split('.')[1]
                v['ros_path'] = ros_path
                v['array'] = True
                v['bounded'] = getattr(type_, 'size', 0)
                if v['proton_type'] != "bytes" and not v['proton_type'].startswith('list_'):
                    v['proton_type'] = f'list_{v['proton_type']}'
                fields[k] = v
        # Array of basic types
        elif proton_type.startswith("list_"):
            fields[proton_name] = {
                "ros_path": ros_path,
                "proton_type": proton_type,
                "array": True,
                "bounded": getattr(type_, 'size', 0)
            }
        # Basic type
        else:
            fields[proton_name] = {
                "ros_path": ros_path,
                "proton_type": proton_type,
                "array": False,
                "bounded": getattr(type_, 'maximum_size', 0)
            }

    return fields

def flatten_service(srv_type):

    flat = {}

    # Resolve request and response messages
    request_cls = getattr(srv_type, "Request")
    response_cls = getattr(srv_type, "Response")

    # Flatten both using your existing flatten_message
    flat["request"] = flatten_message(request_cls)
    flat["response"] = flatten_message(response_cls)

    return flat


def flatten_package_messages(pkg_name):
    """Flatten all messages in a ROS2 package."""
    flat_map = {}

    try:
        msg_module = importlib.import_module(f"{pkg_name}.msg")
    except ModuleNotFoundError:
        return flat_map

    for name in dir(msg_module):
        obj = getattr(msg_module, name)
        if inspect.isclass(obj) and hasattr(obj, "__slots__") and hasattr(obj, "SLOT_TYPES"):
            flat_map[name] = flatten_message(obj)

    return flat_map

def flatten_package_services(pkg_name):
    """Flatten all services in a ROS2 package."""
    flat_map = {}
    try:
        srv_module = importlib.import_module(f"{pkg_name}.srv")
    except ModuleNotFoundError:
        return flat_map

    for name in dir(srv_module):
        obj = getattr(srv_module, name)
        if inspect.isclass(obj) and hasattr(obj, "Request") and hasattr(obj, "Response"):
            flat_map[name] = flatten_service(obj)

    return flat_map

def get_mapping_config(name: str, info: dict) -> dict:
    try:
        mapping_config: dict = {
            ProtonROS2Config.Mapping.PROTON_SIGNAL: name,
            ProtonROS2Config.Mapping.ROS2_PATH: info["ros_path"],
            ProtonROS2Config.Mapping.TYPE: info["proton_type"]
        }
    except KeyError:
        return {}

    if info.get("subpath"):
        mapping_config.update(
            {ProtonROS2Config.Mapping.ROS2_SUBPATH: info["subpath"]}
        )

    if info["array"]:
        mapping_config.update(
            {ProtonROS2Config.Mapping.ROS2_LENGTH: info["bounded"]}
        )
    elif info["bounded"] > 0:
         # String capacity
        pass

    return mapping_config

def get_package_config(package: str) -> dict:
    package = package.strip()
    messages = flatten_package_messages(package)
    services = flatten_package_services(package)

    package_config: dict = {
        ProtonROS2Config.PACKAGE: package,
        ProtonROS2Config.MESSAGES: [],
        ProtonROS2Config.SERVICES: [],
    }

    for message, info in messages.items():
        message_config: dict = {
            ProtonROS2Config.Message.NAME: message,
            ProtonROS2Config.Message.PATH: "msg",
            ProtonROS2Config.Message.MAPPING: []
        }

        for k, v in info.items():
            if v.get('stamp'):
                message_config.update(
                    {ProtonROS2Config.Message.STAMP: v.get('stamp')}
                )
                continue

            message_config[ProtonROS2Config.Message.MAPPING].append(get_mapping_config(k, v))

        package_config[ProtonROS2Config.MESSAGES].append(message_config)

    for service, info in services.items():
        service_config: dict = {
            ProtonROS2Config.Service.NAME: service,
            ProtonROS2Config.Service.PATH: "srv",
            ProtonROS2Config.Service.MAPPING: {
                ProtonROS2Config.Service.REQUEST: [],
                ProtonROS2Config.Service.RESPONSE: []
            }
        }

        for k, v in info['request'].items():
            service_config[ProtonROS2Config.Service.MAPPING][ProtonROS2Config.Service.REQUEST].append(get_mapping_config(k, v))

        for k, v in info['response'].items():
            service_config[ProtonROS2Config.Service.MAPPING][ProtonROS2Config.Service.RESPONSE].append(get_mapping_config(k, v))

        package_config[ProtonROS2Config.SERVICES].append(service_config)

    return package_config

if __name__ == '__main__':
    package = "clearpath_platform_msgs"

    with open(f'{package}.yaml', 'w') as f:
        yaml.dump(get_package_config(package), f)
