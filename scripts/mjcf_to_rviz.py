#!/usr/bin/env python3
"""
Loads the compiled MuJoCo world model (PiperOmron in a robocasa kitchen),
publishes the full body TF tree and camera frustum markers to RViz.

Coordinate frame conventions
-----------------------------
MuJoCo body frames   : right-handed, Z-up — same as ROS REP-103. No correction needed.
MuJoCo camera frames : X right, Y up, looks along -Z  (OpenGL convention)
ROS optical frames   : X right, Y down, looks along +Z (REP-103 optical convention)

To convert a MuJoCo camera quaternion to a ROS optical frame quaternion, compose
with a 180-degree rotation around X:

    q_ros = q_mujoco * Rot(X, 180°)

This flips Y→-Y and Z→-Z, turning (look=-Z, up=+Y) into (look=+Z, up=-Y).
All camera TF frames published here use this corrected convention, so frustums
open along +Z and are directly compatible with ROS camera/image_geometry tools.

Usage:
    roscore &
    python scripts/mjcf_to_rviz.py

In RViz:
    - Fixed Frame: world
    - Add > TF
    - Add > MarkerArray, topic: /mjcf_markers
"""

import numpy as np
import rospy
import tf2_ros
from scipy.spatial.transform import Rotation
from geometry_msgs.msg import TransformStamped, Point
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import ColorRGBA

import robocasa  # noqa: F401
from robocasa.utils.env_utils import create_env

# 180° rotation around X — applied to every camera quaternion to convert
# from MuJoCo camera convention (looks -Z) to ROS optical convention (looks +Z)
_ROT_X180 = Rotation.from_euler("x", 180, degrees=True)


def mj_body_quat_to_ros(q_wxyz):
    """MuJoCo body quaternion (w,x,y,z) -> ROS tf (x,y,z,w). No axis correction needed."""
    return float(q_wxyz[1]), float(q_wxyz[2]), float(q_wxyz[3]), float(q_wxyz[0])


def mj_cam_quat_to_ros(q_wxyz):
    """
    MuJoCo camera quaternion (w,x,y,z) -> ROS optical frame (x,y,z,w).
    Applies 180° rotation around X to convert MuJoCo (-Z look) to ROS (+Z look).
    """
    r = Rotation.from_quat([q_wxyz[1], q_wxyz[2], q_wxyz[3], q_wxyz[0]])
    r_ros = r * _ROT_X180
    q = r_ros.as_quat()  # (x,y,z,w)
    return float(q[0]), float(q[1]), float(q[2]), float(q[3])


def make_transform(parent_frame, child_frame, pos, quat_xyzw, stamp):
    t = TransformStamped()
    t.header.stamp = stamp
    t.header.frame_id = parent_frame
    t.child_frame_id = child_frame
    t.transform.translation.x = float(pos[0])
    t.transform.translation.y = float(pos[1])
    t.transform.translation.z = float(pos[2])
    t.transform.rotation.x = quat_xyzw[0]
    t.transform.rotation.y = quat_xyzw[1]
    t.transform.rotation.z = quat_xyzw[2]
    t.transform.rotation.w = quat_xyzw[3]
    return t


def make_frustum_marker(frame_id, fovy_deg, marker_id, stamp, depth=0.25):
    """
    LINE_LIST frustum in ROS optical frame convention: looks along +Z, X right, Y down.
    fovy_deg is the vertical field of view; aspect ratio assumed 1.0 (square).
    """
    half = np.tan(np.radians(fovy_deg / 2))
    d = depth
    corners = np.array([
        [ half * d, -half * d, d],   # top-right    (X+, Y-, Z+)
        [-half * d, -half * d, d],   # top-left     (X-, Y-, Z+)
        [-half * d,  half * d, d],   # bottom-left  (X-, Y+, Z+)
        [ half * d,  half * d, d],   # bottom-right (X+, Y+, Z+)
    ])
    origin = np.zeros(3)
    lines = []
    for c in corners:
        lines += [origin, c]
    for i in range(4):
        lines += [corners[i], corners[(i + 1) % 4]]

    m = Marker()
    m.header.frame_id = frame_id
    m.header.stamp = stamp
    m.ns = "camera_frustums"
    m.id = marker_id
    m.type = Marker.LINE_LIST
    m.action = Marker.ADD
    m.scale.x = 0.004
    m.color = ColorRGBA(r=1.0, g=1.0, b=0.0, a=1.0)
    m.pose.orientation.w = 1.0
    for pt in lines:
        p = Point()
        p.x, p.y, p.z = float(pt[0]), float(pt[1]), float(pt[2])
        m.points.append(p)
    return m


def make_body_marker(frame_id, marker_id, stamp, is_robot=False):
    """Small sphere at each body origin. Blue = robot/mobile base, green = scene."""
    m = Marker()
    m.header.frame_id = frame_id
    m.header.stamp = stamp
    m.ns = "body_origins"
    m.id = marker_id
    m.type = Marker.SPHERE
    m.action = Marker.ADD
    m.scale.x = m.scale.y = m.scale.z = 0.012
    m.color = ColorRGBA(r=0.2, g=0.6, b=1.0, a=0.8) if is_robot else ColorRGBA(r=0.2, g=0.8, b=0.2, a=0.5)
    m.pose.orientation.w = 1.0
    return m


def make_cam_label(frame_id, cam_name, marker_id, stamp):
    """Yellow text label above each camera frustum."""
    m = Marker()
    m.header.frame_id = frame_id
    m.header.stamp = stamp
    m.ns = "camera_labels"
    m.id = marker_id
    m.type = Marker.TEXT_VIEW_FACING
    m.action = Marker.ADD
    m.scale.z = 0.12
    m.color = ColorRGBA(r=1.0, g=1.0, b=0.0, a=1.0)
    m.pose.position.z = 0.18
    m.pose.orientation.w = 1.0
    m.text = cam_name
    return m


def make_body_label(frame_id, body_name, marker_id, stamp):
    """White text label at each body origin."""
    m = Marker()
    m.header.frame_id = frame_id
    m.header.stamp = stamp
    m.ns = "body_labels"
    m.id = marker_id
    m.type = Marker.TEXT_VIEW_FACING
    m.action = Marker.ADD
    m.scale.z = 0.06
    m.color = ColorRGBA(r=0.9, g=0.9, b=0.9, a=0.8)
    m.pose.position.z = 0.02
    m.pose.orientation.w = 1.0
    m.text = body_name
    return m


def main():
    rospy.init_node("mjcf_rviz_validator", anonymous=False)

    rospy.loginfo("Creating PiperOmron kitchen env (no rendering)...")
    env = create_env(
        env_name="PickPlaceCounterToCabinet",
        robots="PiperOmron",
        render_onscreen=False,
    )
    env.reset()

    rs_model = env.sim.model    # robosuite MjModel wrapper
    mj_model = rs_model._model  # raw mujoco.MjModel

    rospy.loginfo(
        f"Model: {mj_model.nbody} bodies, {mj_model.njnt} joints, "
        f"{mj_model.ncam} cameras, {mj_model.ngeom} geoms"
    )

    broadcaster = tf2_ros.TransformBroadcaster()
    marker_pub = rospy.Publisher("/mjcf_markers", MarkerArray, queue_size=1, latch=True)

    # Pre-collect entries once — model is static
    body_entries = []
    for body_id in range(mj_model.nbody):
        body_name = rs_model._body_id2name.get(body_id, f"body_{body_id}")
        if body_id == 0:
            parent_frame = "world"
        else:
            parent_id = int(mj_model.body_parentid[body_id])
            parent_name = rs_model._body_id2name.get(parent_id, f"body_{parent_id}")
            parent_frame = f"body_{parent_name}"
        is_robot = "robot" in body_name or "mobilebase" in body_name
        body_entries.append((
            parent_frame,
            f"body_{body_name}",
            mj_model.body_pos[body_id],
            mj_body_quat_to_ros(mj_model.body_quat[body_id]),
            body_name,
            is_robot,
        ))

    cam_entries = []
    for cam_id in range(mj_model.ncam):
        cam_name = rs_model._camera_id2name.get(cam_id, f"cam_{cam_id}")
        body_id = int(mj_model.cam_bodyid[cam_id])
        body_name = rs_model._body_id2name.get(body_id, f"body_{body_id}")
        fovy = float(mj_model.cam_fovy[cam_id])
        q_ros = mj_cam_quat_to_ros(mj_model.cam_quat[cam_id])
        cam_entries.append((
            cam_id, cam_name,
            f"body_{body_name}",
            mj_model.cam_pos[cam_id],
            q_ros,
            fovy,
        ))
        rospy.loginfo(f"  cam '{cam_name}' -> parent '{body_name}' | fovy={fovy:.1f}°")

    rospy.loginfo(
        f"Publishing {len(body_entries)} body + {len(cam_entries)} camera frames at 10 Hz. "
        "RViz: Fixed Frame=world, add TF + MarkerArray on /mjcf_markers."
    )

    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        stamp = rospy.Time.now()
        tf_msgs = []
        markers = MarkerArray()

        for parent_frame, child_frame, pos, q_xyzw, body_name, is_robot in body_entries:
            tf_msgs.append(make_transform(parent_frame, child_frame, pos, q_xyzw, stamp))
            markers.markers.append(make_body_marker(child_frame, len(markers.markers), stamp, is_robot))
            markers.markers.append(make_body_label(child_frame, body_name, mj_model.nbody * 2 + len(cam_entries) * 2 + len(markers.markers), stamp))

        for cam_id, cam_name, parent_frame, pos, q_xyzw, fovy in cam_entries:
            cam_frame = f"cam_{cam_name}"
            tf_msgs.append(make_transform(parent_frame, cam_frame, pos, q_xyzw, stamp))
            base_id = mj_model.nbody + cam_id * 2
            markers.markers.append(make_frustum_marker(cam_frame, fovy, base_id, stamp))
            markers.markers.append(make_cam_label(cam_frame, cam_name, base_id + 1, stamp))

        broadcaster.sendTransform(tf_msgs)
        marker_pub.publish(markers)
        rate.sleep()


if __name__ == "__main__":
    main()
