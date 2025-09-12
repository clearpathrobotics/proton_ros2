import importlib
import inspect
from rosidl_runtime_py.utilities import get_message
from rosidl_parser.definition import (
    AbstractSequence, Array, AbstractString,
    NamespacedType, BasicType
)


def format_type_info(field_type):
    """Format ROS2 field type info (arrays, strings, primitives, nested)."""
    if isinstance(field_type, AbstractSequence):
        return f"sequence<{format_type_info(field_type.value_type)}>"
    elif isinstance(field_type, Array):
        return f"array<{format_type_info(field_type.value_type)}, size={field_type.size}>"
    elif isinstance(field_type, AbstractString):
        if field_type.has_maximum_size():
            return f"string<max_length={field_type.maximum_size}>"
        return "string"
    elif isinstance(field_type, NamespacedType):
        return "/".join(field_type.namespaces + [field_type.name])
    elif isinstance(field_type, BasicType):
        return field_type.typename
    return str(field_type)


def inspect_message_type(msg_type, indent=0, seen=None):
    """Recursively introspect a ROS2 message class."""
    if seen is None:
        seen = set()

    # if msg_type in seen:
    #     print(" " * indent + f"(Already inspected {msg_type.__name__})")
    #     return
    seen.add(msg_type)

    print(" " * indent + f"Message: {msg_type.__module__}.{msg_type.__name__}")

    field_types = msg_type.SLOT_TYPES
    for field_name, field_type in zip(msg_type.__slots__, field_types):
        prefix = " " * (indent + 2)
        print(f"{prefix}{field_name}: {format_type_info(field_type)}")

        # --- NEW: create a dummy instance to check nested fields ---
        instance = msg_type()
        value = getattr(instance, field_name)

        # If this field is another message type (not primitive or list of primitives)
        if hasattr(value, "__slots__") and hasattr(value, "SLOT_TYPES"):
            inspect_message_type(type(value), indent + 4, seen)

        # If it's a sequence of nested messages
        elif isinstance(field_type, AbstractSequence):
            elem_type = field_type.value_type
            if isinstance(elem_type, NamespacedType):
                try:
                    nested_type = get_message(
                        f"{elem_type.namespaces[0]}.msg.{elem_type.name}"
                    )
                    inspect_message_type(nested_type, indent + 4, seen)
                except Exception:
                    pass


def inspect_package_messages(package_name):
    """Inspect all messages in a ROS2 package recursively."""
    try:
        msg_module = importlib.import_module(f"{package_name}.msg")
    except ModuleNotFoundError:
        print(f"Package '{package_name}' not found or has no 'msg' module.")
        return

    for name, obj in inspect.getmembers(msg_module, inspect.isclass):
        if hasattr(obj, "get_fields_and_field_types"):
            inspect_message_type(obj)


# ---------------------------
# Example usage:
# ---------------------------
if __name__ == "__main__":
    inspect_package_messages("clearpath_platform_msgs")
