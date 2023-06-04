# coding=utf-8

"""
点云处理函数,将ros中的PointCdoud2格式转化为np.ndarray格式
"""

import struct
import rospy
from sensor_msgs import point_cloud2
import std_msgs.msg as std_msgs
from sensor_msgs.msg import PointCloud2, PointField

import numpy as np

# prefix to the names of dummy fields we add to get byte alignment correct. this needs to not
# clash with any actual field names
DUMMY_FIELD_PREFIX = "__"

# PointField的格式和numpy中格式的映射
type_mappings = [
    (PointField.INT8, np.dtype("int8")),
    (PointField.UINT8, np.dtype("uint8")),
    (PointField.INT16, np.dtype("int16")),
    (PointField.UINT16, np.dtype("uint16")),
    (PointField.INT32, np.dtype("int32")),
    (PointField.UINT32, np.dtype("uint32")),
    (PointField.FLOAT32, np.dtype("float32")),
    (PointField.FLOAT64, np.dtype("float64")),
]
pftype_to_nptype = dict(type_mappings)
nptype_to_pftype = dict((nptype, pftype) for pftype, nptype in type_mappings)

# PointField格式的字节大小
pftype_sizes = {
    PointField.INT8: 1,
    PointField.UINT8: 1,
    PointField.INT16: 2,
    PointField.UINT16: 2,
    PointField.INT32: 4,
    PointField.UINT32: 4,
    PointField.FLOAT32: 4,
    PointField.FLOAT64: 8,
}


def _float2rgb(x):
    rgb = struct.unpack('I', struct.pack('f', x))[0]
    b = (rgb >> 16) & 0x0000ff
    g = (rgb >> 8) & 0x0000ff
    r = (rgb) & 0x0000ff
    return r, g, b


class PointCloud2Utils:
    """
    点云消息PointCloud2处理工具类
    """

    @staticmethod
    def pointcloud2_to_dtype(cloud_msg):
        """
        Convert a list of PointFields to a numpy record datatype.
        """
        offset = 0
        np_dtype_list = []
        for f in cloud_msg.fields:
            while offset < f.offset:
                # might be extra padding between fields
                np_dtype_list.append(
                    ("%s%d" % (DUMMY_FIELD_PREFIX, offset), np.uint8))
                offset += 1
            np_dtype_list.append((f.name, pftype_to_nptype[f.datatype]))
            offset += pftype_sizes[f.datatype]

        # might be extra padding between points
        while offset < cloud_msg.point_step:
            np_dtype_list.append(
                ("%s%d" % (DUMMY_FIELD_PREFIX, offset), np.uint8))
            offset += 1

        return np_dtype_list

    @staticmethod
    def arr_to_fields(cloud_arr):
        """
        Convert a numpy record datatype into a list of PointFields.
        """
        fields = []
        for field_name in cloud_arr.dtype.names:
            np_field_type, field_offset = cloud_arr.dtype.fields[field_name]
            pf = PointField()
            pf.name = field_name
            pf.datatype = nptype_to_pftype[np_field_type]
            pf.offset = field_offset
            pf.count = 1  # is this ever more than one?
            fields.append(pf)
        return fields

    @staticmethod
    def pointcloud2_to_array(cloud_msg, split_rgb=False):
        """
        Converts a rospy PointCloud2 message to a numpy recordarray

        Reshapes the returned array to have shape (height, width), even if the height is 1.

        The reason for using np.fromstring rather than struct.unpack is speed... especially
        for large point clouds, this will be <much> faster.
        """
        # construct a numpy record type equivalent to the point type of this cloud
        dtype_list = PointCloud2Utils.pointcloud2_to_dtype(cloud_msg)

        # parse the cloud into an array
        cloud_arr = np.fromstring(cloud_msg.data, dtype_list)

        # remove the dummy fields that were added
        cloud_arr = cloud_arr[
            [
                fname
                for fname, _type in dtype_list
                if not (fname[: len(DUMMY_FIELD_PREFIX)] == DUMMY_FIELD_PREFIX)
            ]
        ]

        if split_rgb:
            cloud_arr = PointCloud2Utils.split_rgb_field(cloud_arr)

        return np.reshape(cloud_arr, (cloud_msg.height, cloud_msg.width))

    @staticmethod
    def array_to_pointcloud2(cloud_arr, stamp=None, frame_id=None, merge_rgb=False):
        """Converts a numpy record array to a sensor_msgs.msg.PointCloud2."""
        if merge_rgb:
            cloud_arr = PointCloud2Utils.merge_rgb_fields(cloud_arr)

        # make it 2d (even if height will be 1)
        cloud_arr = np.atleast_2d(cloud_arr)

        cloud_msg = PointCloud2()

        if stamp is not None:
            cloud_msg.header.stamp = stamp
        if frame_id is not None:
            cloud_msg.header.frame_id = frame_id
        cloud_msg.height = cloud_arr.shape[0]
        cloud_msg.width = cloud_arr.shape[1]
        cloud_msg.fields = PointCloud2Utils.arr_to_fields(cloud_arr)
        cloud_msg.is_bigendian = False  # assumption
        cloud_msg.point_step = cloud_arr.dtype.itemsize
        cloud_msg.row_step = cloud_msg.point_step * cloud_arr.shape[1]
        cloud_msg.is_dense = all(
            [np.isfinite(cloud_arr[fname]).all()
             for fname in cloud_arr.dtype.names]
        )
        cloud_msg.data = cloud_arr.tostring()
        return cloud_msg

    @staticmethod
    def merge_rgb_fields(cloud_arr):
        """
        Takes an array with named np.uint8 fields 'r', 'g', and 'b', and returns an array in
        which they have been merged into a single np.float32 'rgb' field. The first byte of this
        field is the 'r' uint8, the second is the 'g', uint8, and the third is the 'b' uint8.

        This is the way that pcl likes to handle RGB colors for some reason.
        """
        r = np.asarray(cloud_arr["r"], dtype=np.uint32)
        g = np.asarray(cloud_arr["g"], dtype=np.uint32)
        b = np.asarray(cloud_arr["b"], dtype=np.uint32)
        rgb_arr = np.array((r << 16) | (g << 8) | (b << 0), dtype=np.uint32)

        # not sure if there is a better way to do this. i'm changing the type of the array
        # from uint32 to float32, but i don't want any conversion to take place -jdb
        rgb_arr.dtype = np.float32

        # create a new array, without r, g, and b, but with rgb float32 field
        new_dtype = []
        for field_name in cloud_arr.dtype.names:
            field_type, field_offset = cloud_arr.dtype.fields[field_name]
            if field_name not in ("r", "g", "b"):
                new_dtype.append((field_name, field_type))
        new_dtype.append(("rgb", np.float32))
        new_cloud_arr = np.zeros(cloud_arr.shape, new_dtype)

        # fill in the new array
        for field_name in new_cloud_arr.dtype.names:
            if field_name == "rgb":
                new_cloud_arr[field_name] = rgb_arr
            else:
                new_cloud_arr[field_name] = cloud_arr[field_name]

        return new_cloud_arr

    @staticmethod
    def split_rgb_field(cloud_arr):
        """
        Takes an array with a named 'rgb' float32 field, and returns an array in which
        this has been split into 3 uint 8 fields: 'r', 'g', and 'b'.

        (pcl stores rgb in packed 32 bit floats)
        """
        rgb_arr = cloud_arr["rgb"].copy()
        rgb_arr.dtype = np.uint32
        r = np.asarray((rgb_arr >> 16) & 255, dtype=np.uint8)
        g = np.asarray((rgb_arr >> 8) & 255, dtype=np.uint8)
        b = np.asarray(rgb_arr & 255, dtype=np.uint8)

        # create a new array, without rgb, but with r, g, and b fields
        new_dtype = []
        for field_name in cloud_arr.dtype.names:
            field_type, field_offset = cloud_arr.dtype.fields[field_name]
            if not field_name == "rgb":
                new_dtype.append((field_name, field_type))
        new_dtype.append(("r", np.uint8))
        new_dtype.append(("g", np.uint8))
        new_dtype.append(("b", np.uint8))
        new_cloud_arr = np.zeros(cloud_arr.shape, new_dtype)

        # fill in the new array
        for field_name in new_cloud_arr.dtype.names:
            if field_name == "r":
                new_cloud_arr[field_name] = r
            elif field_name == "g":
                new_cloud_arr[field_name] = g
            elif field_name == "b":
                new_cloud_arr[field_name] = b
            else:
                new_cloud_arr[field_name] = cloud_arr[field_name]
        return new_cloud_arr

    @staticmethod
    def get_xyz_points(cloud_array, remove_nans=True, dtype=np.float):
        """
        抽取出点云信息中的 xyz信息, 返回一个Nx3的点云矩阵
        """
        # remove crap points
        if remove_nans:
            mask = (
                np.isfinite(cloud_array["x"])
                & np.isfinite(cloud_array["y"])
                & np.isfinite(cloud_array["z"])
            )
            cloud_array = cloud_array[mask]

        # pull out x, y, and z values
        points = np.zeros(list(cloud_array.shape) + [3], dtype=dtype)
        points[..., 0] = cloud_array["x"]
        points[..., 1] = cloud_array["y"]
        points[..., 2] = cloud_array["z"]

        return points

    @staticmethod
    def pointcloud2_to_xyz_array(cloud_msg, remove_nans=True):
        return PointCloud2Utils.get_xyz_points(
            PointCloud2Utils.pointcloud2_to_array(cloud_msg),
            remove_nans=remove_nans,
        )

    @staticmethod
    def xyz_array_to_pointcloud2(cloud_arr, parent_frame):
        """将输入的np.ndarray格式的点云转化为sensor_msgs/PointCloud2格式

        Args:
            cloud_arr (np.ndarray): 输入点云, shape=(num_points, 3)
            parent_frame (str): PointCloud2格式信息的父节点名称

        Returns:
            sensor_msgs/PointCloud2: 转换后的点云信息
        """
        ros_dtype = PointField.FLOAT32
        src_dtype = np.float32
        itemsize = np.dtype(src_dtype).itemsize
        data = cloud_arr.astype(src_dtype).tobytes()

        fields = [PointField(name=n, offset=i * itemsize, datatype=ros_dtype, count=1)
                  for i, n in enumerate('xyz')
                  ]

        header = std_msgs.Header(frame_id=parent_frame, stamp=rospy.Time.now())

        return PointCloud2(
            header=header,
            height=1,
            width=cloud_arr.shape[0],
            is_dense=np.isfinite(cloud_arr).all(),
            is_bigendian=False,
            fields=fields,
            point_step=(itemsize * 3),
            row_step=(itemsize * 3 * cloud_arr.shape[0]),
            data=data
        )

    @staticmethod
    def pointcloud2_to_xyzrgb_array(cloud_msg):
        points = point_cloud2.read_points(cloud_msg)
        points_list = np.asarray(list(points))
        points_arr = np.asarray(points_list)

        # Unpack RGB color info
        _float2rgb_vectorized = np.vectorize(_float2rgb)
        r, g, b = _float2rgb_vectorized(points_arr[:, 3])

        # Concatenate and Reshape
        # insert blank 3rd dimension (for concatenation)
        r = np.expand_dims(r, 1)
        g = np.expand_dims(g, 1)
        b = np.expand_dims(b, 1)
        points_rgb = np.concatenate((points_arr[:, 0: 3], r, g, b), axis=1)

        # remove nan
        mask = np.isfinite(points_rgb[:, 0]) & np.isfinite(
            points_rgb[:, 1]) & np.isfinite(points_rgb[:, 2])

        return points_rgb[mask]

    @staticmethod
    def xyzrgb_array_to_pointcloud2(points, parent_frame):
        """将np.ndarray表示的点云转变为sensor_msgs/PointCloud2格式

        Args:
            points (np.ndarray (n,6)): 点云, 颜色信息[0~1]
            parent_frame(str): 坐标系名称
        """
        points[:, [3, 5]] = points[:, [5, 3]]
        ros_dtype = PointField.FLOAT32
        dtype = np.float32
        itemsize = np.dtype(dtype).itemsize
        data = points.astype(dtype).tobytes()

        fields = [PointField(
            name=n, offset=i * itemsize, datatype=ros_dtype, count=1)
            for i, n in enumerate('xyzrgb')]

        header = std_msgs.Header(frame_id=parent_frame, stamp=rospy.Time.now())

        return PointCloud2(
            header=header,
            height=1,
            width=points.shape[0],
            is_dense=False,
            is_bigendian=False,
            fields=fields,
            point_step=(itemsize * 6),
            row_step=(itemsize * 6 * points.shape[0]),
            data=data
        )
