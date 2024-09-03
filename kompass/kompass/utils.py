from ros_sugar.utils import (
    IncompatibleSetup,
    IntEnum,
    action_handler,
    camel_to_snake_case,
    component_action,
    component_fallback,
    get_methods_with_decorator,
    has_decorator,
    launch_action,
    log_srv,
)
from sensor_msgs.msg import PointCloud2
from tf2_ros import TransformStamped
import PyKDL
import struct


__all__ = [
    "IncompatibleSetup",
    "IntEnum",
    "action_handler",
    "camel_to_snake_case",
    "component_action",
    "component_fallback",
    "get_methods_with_decorator",
    "has_decorator",
    "launch_action",
    "log_srv",
]


def read_pc_points(cloud_msg: PointCloud2):
    """Read sensor_msgs PointCloud2 ROS2 message points [(x, y, z)]

    :param cloud_msg: Message
    :type cloud_msg: PointCloud2
    :raises TypeError: If message data is bigendian

    :yield: One point in the cloud (x, y, z)
    :rtype: tuple
    """
    if cloud_msg.is_bigendian:
        raise TypeError("Bigendian data is not supported")

    point_step = cloud_msg.point_step
    row_step = cloud_msg.row_step
    data = cloud_msg.data
    height = cloud_msg.height
    width = cloud_msg.width

    # Find the offset for x, y, z in the point structure
    x_offset = None
    y_offset = None
    z_offset = None

    for field in cloud_msg.fields:
        if field.name == "x":
            x_offset = field.offset
        elif field.name == "y":
            y_offset = field.offset
        elif field.name == "z":
            z_offset = field.offset

    assert (
        x_offset is not None and y_offset is not None and z_offset is not None
    ), "Offsets for x, y, z are not found"

    # Iterate over the points in the data
    for row in range(height):
        for col in range(width):
            # Calculate the starting byte for this point in the data array
            point_start = row * row_step + col * point_step

            # Extract x, y, z values using the offsets
            x = struct.unpack_from("f", data, point_start + x_offset)[0]
            y = struct.unpack_from("f", data, point_start + y_offset)[0]
            z = struct.unpack_from("f", data, point_start + z_offset)[0]

            yield (x, y, z)


def read_pc_points_with_tf(cloud_msg: PointCloud2, transform: TransformStamped):
    """Read sensor_msgs PointCloud2 ROS2 message points and apply transformation

    :param cloud_msg: Message
    :type cloud_msg: PointCloud2
    :param transform: Transformation
    :type transform: TransformStamped
    :yield: One transformed point in the cloud (x, y, z)
    :rtype: tuple
    """
    tf_kdl = PyKDL.Frame(
        PyKDL.Rotation.Quaternion(
            transform.transform.rotation.x,
            transform.transform.rotation.y,
            transform.transform.rotation.z,
            transform.transform.rotation.w,
        ),
        PyKDL.Vector(
            transform.transform.translation.x,
            transform.transform.translation.y,
            transform.transform.translation.z,
        ),
    )
    for p_in in read_pc_points(cloud_msg):
        p_out = tf_kdl * PyKDL.Vector(p_in[0], p_in[1], p_in[2])
        yield p_out
