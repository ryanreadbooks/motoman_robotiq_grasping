# coding=utf-8

import rospy
import numpy as np
import geometry_msgs.msg as gmsg
import tf2_ros
import tf2_geometry_msgs
import tf.transformations as tft

from inference import GripperFrame
import pytransform3d.rotations as pr


# TF2的缓冲区
_tfBuffer = None
# 监听器
_listener = None


def _init_tf():
    """
    初始化TF缓冲区和监听器
    """
    global _tfBuffer, _listener
    _tfBuffer = tf2_ros.Buffer()
    _listener = tf2_ros.TransformListener(_tfBuffer)


def quaternion_to_list(quaternion):
    """
    gmsg.Quaternion四元数转换成list
    """
    return [quaternion.x, quaternion.y, quaternion.z, quaternion.w]


def list_to_quaternion(l):
    """list转换成gmsg.Quaternion四元数

    Args:
        l (list): [x,y,z,w]表示的四元数

    Returns:
        geometry_msgs/Quaternion: 四元数
    """
    q = gmsg.Quaternion()
    q.x = l[0]
    q.y = l[1]
    q.z = l[2]
    q.w = l[3]
    return q


def convert_pose(pose, from_frame, to_frame) -> gmsg.Pose:
    """将一个pose从from_frame转换到to_frame下

    Args:
        pose (geometry_msgs.msg/Pose): 需要转换的pose
        from_frame (str): 原始的坐标系,也就是pose是在哪个frame下定义的
        to_frame (str): 希望转到的坐标系

    Returns:
        geometry_msgs.msg/Pose: 转换到to_frame的pose
    """
    
    global _tfBuffer, _listener

    if _tfBuffer is None or _listener is None:
        _init_tf()

    try:
        trans = _tfBuffer.lookup_transform(to_frame, from_frame, rospy.Time(0), rospy.Duration(1.0))
    except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
        print(e)
        rospy.logerr('FAILED TO GET TRANSFORM FROM %s to %s' % (to_frame, from_frame))
        return None

    spose = gmsg.PoseStamped()
    spose.pose = pose
    spose.header.stamp = rospy.Time().now
    spose.header.frame_id = from_frame

    p2 = tf2_geometry_msgs.do_transform_pose(spose, trans)

    return p2.pose


def current_robot_pose(reference_frame, base_frame):
    """获取机器人当前在reference_frame下的pose

    Args:
        reference_frame (str): 参考系的frame_id
        base_frame (str): 基坐标系

    Returns:
        geometry_msgs.msg/Pose: 转换到reference_frame的pose
    """ 
    # Create Pose
    p = gmsg.Pose()
    p.orientation.w = 1.0

    # Transforms robots current pose to the base reference frame
    return convert_pose(p, base_frame, reference_frame)


def publish_stamped_transform(stamped_transform, seconds=1):
    """广播TF变换

    Args:
        stamped_transform (geometry_msgs/TransformStamped): 待发布的变换
        seconds (float): 持续发布变换多少时长
    """

    # Create broadcast node
    br = tf2_ros.TransformBroadcaster()

    # Publish once first.
    stamped_transform.header.stamp = rospy.Time.now()
    br.sendTransform(stamped_transform)
    if seconds == 0:
        stamped_transform.header.stamp = rospy.Time.now()
        br.sendTransform(stamped_transform)
    else:
        # Publish transform for set time.
        i = 0
        iterations = seconds / 0.05
        while not rospy.is_shutdown() and i < iterations:
            stamped_transform.header.stamp = rospy.Time.now()
            br.sendTransform(stamped_transform)
            rospy.sleep(0.05)
            i += 1


def publish_transform(transform, reference_frame, name, seconds=1):
    """发布一个TF变换

    Args:
        transform (geometry_msgs/Transform): 需要发布的geometry_msgs/Transform
        reference_frame (str): 要发布的transform的frame_id
        seconds (float): 发布持续时间
    """
    # Create a stamped_transform and store the transform in it
    st = gmsg.TransformStamped()
    st.transform = transform
    st.header.frame_id = reference_frame
    st.child_frame_id = name

    # Call the publish_stamped_transform function
    publish_stamped_transform(st, seconds)


def publish_pose_as_transform(pose, reference_frame, name, seconds=1):
    """发布一个从reference_frame到name之间的变换关系
    
    Args:
        pose (geometry_msgs.msg/Pose): 需要被发布的pose, 会被转换成Transform再发布
        reference_frame (str): 被转换的pose的父frame_id
        name (str): 被变换的pose的名称
        seconds (float): 发布持续时间
    """

    # Create a broadcast node and a stamped transform to broadcast
    t = gmsg.TransformStamped()

    # Prepare broadcast message
    t.header.frame_id = reference_frame
    t.child_frame_id = name

    # Copy in pose values to transform
    t.transform.translation = pose.position
    t.transform.rotation = pose.orientation

    # Call the publish_stamped_transform function
    publish_stamped_transform(t, seconds)


def publish_tf_quaterion_as_transform(translation, quaternion, reference_frame, name, seconds=1):
    qm = gmsg.Transform()
    qm.translation.x = translation[0]
    qm.translation.y = translation[1]
    qm.translation.z = translation[2]
    qm.rotation.x = quaternion[0]
    qm.rotation.y = quaternion[1]
    qm.rotation.z = quaternion[2]
    qm.rotation.w = quaternion[3]
    publish_transform(qm, reference_frame, name, seconds)


def align_pose_orientation_to_frame(from_pose, from_reference_frame, to_reference_frame):
    """将from_pose的orientation更新为对齐to_reference_frame的orientation

    Args:
        from_pose (geometry_msgs.msg/Pose): 要对齐的pose
        from_reference_frame (str): 这个pose是在哪个frame下的
        to_reference_frame (str): 想要pose对齐哪个frame
    """
    # Create transform
    p = gmsg.Pose()
    p.orientation.w = 1.0

    # Convert reference frame orientation from -> to
    pose_orientation = convert_pose(p, to_reference_frame, from_reference_frame)

    # Copy orientation to pose.
    from_pose.orientation = pose_orientation.orientation

    return from_pose


def generate_grasp_6dpose(center, direction, angle):
    """根据相机坐标系下的抓取点, 抓取方向和末端旋转角度计算在机械臂基坐标系下的抓取位姿

    Args:
        center (np.ndarray (3,)): 在相机坐标系下的抓取点坐标, (x,y,z)
        direction (np.ndarray (3,)): 在相机坐标系下的抓取方向向量, (dx, dy, dz), 这个方向一般都是指向物体的
        angle (float): 法兰盘旋转弧度数rad, 即绕着direction作为旋转轴的旋转弧度数

    Returns:
        geometry_msgs.msg/Pose: 抓取位姿在机械臂基座标系下的表示
    """
    # 初始化在相机坐标系的位姿
    gp_cam = gmsg.Pose()
    gp_cam.position.x = center[0]
    gp_cam.position.y = center[1]
    gp_cam.position.z = center[2]
    gp_cam.orientation.w = 1

    # 得到在相机坐标系下的grasp frame的6d pose
    gripper_frame_cam = GripperFrame.init(center, direction, 0)
    grasp_pose_cam = gripper_frame_cam.to_6dpose()  # shape=(4, 4)
    grasp_pose_cam_rotmat = grasp_pose_cam[:3, :3]

    # 绕着x轴(ros中的x轴)旋转欧拉角对应的四元数
    angle_qx, angle_qy, angle_qz, angle_qw = tft.quaternion_from_euler(angle, 0, 0) 
    # 对应的旋转矩阵
    angle_rotmat = pr.matrix_from_quaternion([angle_qw, angle_qx, angle_qy, angle_qz])
    # 右乘得到总体旋转姿态量
    # - (先通过grasp_pose_cam_rotmat从相机坐标系变换到目标坐标系,
    # - 随后绕着目标坐标系的某个轴进行旋转,所以右乘旋转矩阵angle_rotmat)
    total_rotmat = grasp_pose_cam_rotmat @ angle_rotmat

    # 相机坐标系转换到base_link坐标系
    gp_base = convert_pose(gp_cam, 'camera_color_optical_frame', 'base_link')
    qw, qx, qy, qz = pr.quaternion_from_matrix(total_rotmat)
    gp_base.orientation.x = qx
    gp_base.orientation.y = qy
    gp_base.orientation.z = qz
    gp_base.orientation.w = qw

    return gp_base


# ?! 未测试确认这个函数的正确性
def generate_grasp_6dpose_from_obj_6dpose(pose):
    """从物体在相机坐标系下的6D位姿生成抓取位姿

    Args:
        pose (np.ndarray (4,4)): 物体6D位姿
    """
    translation = pose[:3, -1]
    rotmat = pose[:3, :3]
    direction = -rotmat[:, -1]
    # 得到两个向量之间的旋转矩阵
    diff_rot_mat = pr.matrix_from_rotor(pr.rotor_from_two_directions(np.array([0., 0., 1.]), -direction))
    # 计算两个坐标系(这两个坐标系的z轴是重合的)之间的角度差
    angle = pr.angle_between_vectors(rotmat[:,0], diff_rot_mat[:,0])    #　ｘ分量之间的角度差
    return generate_grasp_6dpose(translation, direction, angle)


def publish_grasp_candidate(center, direction, angle, gp_frame_name):
    """将相机坐标系的抓取点, 抓取方向和末端旋转角度变化到机械臂基座标系后发布

    Args:
        center (np.ndarray (3,)): 在相机坐标系下的抓取点坐标, (x,y,z)
        direction (np.ndarray (3,)): 在相机坐标系下的抓取方向向量, (dx, dy, dz), 这个方向一般都是指向物体的
        angle (float): 绕着direction作为旋转轴的旋转弧度数rad
    """

    grasp_pose_wrt_base = generate_grasp_6dpose(center, direction, angle)
    publish_pose_as_transform(grasp_pose_wrt_base, 'base_link', gp_frame_name, 0)
    return grasp_pose_wrt_base


def publish_grasp_candidate_using_6d_pose(grasp_rot, grasp_trans, gp_frame_name):
    """将在相机坐标系下的抓取位姿发布出去, 位姿用一个3x3的旋转矩阵和一个3维的平移向量表示

    Args:
        grasp_rot (np.ndarray): 抓取位姿中的姿态量,用3x3的矩阵表示
        grasp_trans (np.ndarray): 抓取位姿中的平移量
        gp_frame_name (str): 发布的坐标系的名称
    """
    assert grasp_rot.shape == (3, 3), '3x3 rotation matrix is required for grasp_rot, but got {}'.format(grasp_rot.shape)
    grasp_pose_point = gmsg.Point(x=grasp_trans[0], y=grasp_trans[1], z=grasp_trans[2])
    grasp_rot_44 = np.zeros((4, 4))
    grasp_rot_44[-1, -1] = 1
    grasp_rot_44[:3, :3] = grasp_rot
    grasp_quat_np = tft.quaternion_from_matrix(grasp_rot_44)    # quaternion in np.ndarray format, [x,y,z,w]
    grasp_quat = gmsg.Quaternion(x=grasp_quat_np[0], y=grasp_quat_np[1], z=grasp_quat_np[2], w=grasp_quat_np[3])
    grasp_pose = gmsg.Pose(position=grasp_pose_point, orientation=grasp_quat)

    publish_pose_as_transform(grasp_pose, 'camera_color_optical_frame', gp_frame_name, 0)


def get_rotmat_around_x(rx):
    """
    获取绕x轴旋转的旋转矩阵, 在yz平面逆时针
    :param rx: 旋转角度,单位rad
    :return:
    """
    return np.array([[1, 0, 0],
                     [0, np.cos(rx), -np.sin(rx)],
                     [0, np.sin(rx), np.cos(rx)]])


def get_rotmat_around_y(ry):
    """
    获取绕y轴旋转的旋转矩阵, 在zx平面逆时针
    :param ry: 旋转角度,单位rad
    :return:
    """
    return np.array([[np.cos(ry), 0, np.sin(ry)],
                     [0, 1, 0],
                     [-np.sin(ry), 0, np.cos(ry)]])


def get_rotmat_around_z(rz):
    """
    获取绕z轴旋转的旋转矩阵, 在xy平面逆时针
    :param rz: 旋转角度,单位rad
    :return:
    """
    return np.array([[np.cos(rz), -np.sin(rz), 0],
                     [np.sin(rz), np.cos(rz), 0],
                     [0, 0, 1]])

