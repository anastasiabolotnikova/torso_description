#!/usr/bin/env python3
# Publishes RSDF planar_surface polygons as RViz markers, for previewing an RSDF file
# without needing a RobotModule/controller to load it.
#
# Usage:
#   ros2 run robot_state_publisher robot_state_publisher --ros-args --params-file <rsp_params.yaml>  # mesh
#   python3 visualize_rsdf_surfaces.py <path-to-rsdf-file>                                           # surfaces
#   rviz2   # add a MarkerArray display on topic /rsdf_surfaces, Fixed Frame = the surfaces' link name
import sys
import xml.etree.ElementTree as ET

import numpy as np
import rclpy
from geometry_msgs.msg import Point
from rclpy.node import Node
from visualization_msgs.msg import Marker, MarkerArray


def rpy_to_matrix(roll, pitch, yaw):
    cr, sr = np.cos(roll), np.sin(roll)
    cp, sp = np.cos(pitch), np.sin(pitch)
    cy, sy = np.cos(yaw), np.sin(yaw)
    Rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
    Ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    Rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    return Rz @ Ry @ Rx


def parse_rsdf(path):
    root = ET.parse(path).getroot()
    surfaces = []
    for surf in root.findall("planar_surface"):
        name = surf.get("name")
        link = surf.get("link")
        origin = surf.find("origin")
        xyz = np.array([float(v) for v in origin.get("xyz").split()])
        rpy = np.array([float(v) for v in origin.get("rpy").split()])
        rot = rpy_to_matrix(*rpy)
        points_2d = [
            tuple(float(v) for v in p.get("xy").split())
            for p in surf.find("points").findall("point")
        ]
        points_3d = [xyz + rot @ np.array([x, y, 0.0]) for x, y in points_2d]
        surfaces.append((name, link, points_3d))
    return surfaces


class RsdfMarkerPublisher(Node):
    def __init__(self, rsdf_path):
        super().__init__("rsdf_surface_visualizer")
        self.surfaces = parse_rsdf(rsdf_path)
        self.pub = self.create_publisher(MarkerArray, "rsdf_surfaces", 10)
        self.timer = self.create_timer(1.0, self.publish_markers)
        self.get_logger().info(
            f"Publishing {len(self.surfaces)} surface(s) from {rsdf_path} on /rsdf_surfaces"
        )

    def publish_markers(self):
        array = MarkerArray()
        stamp = self.get_clock().now().to_msg()
        for i, (name, link, points_3d) in enumerate(self.surfaces):
            center = np.mean(points_3d, axis=0)

            if len(points_3d) < 3:
                # A single point (or a degenerate line) has no polygon to fill/outline --
                # draw a small sphere at each point instead so it's still visible.
                for k, p in enumerate(points_3d):
                    dot = Marker()
                    dot.header.frame_id = link
                    dot.header.stamp = stamp
                    dot.ns = "rsdf_surfaces"
                    dot.id = 1000 * i + k
                    dot.type = Marker.SPHERE
                    dot.action = Marker.ADD
                    dot.scale.x = dot.scale.y = dot.scale.z = 0.02
                    dot.color.r, dot.color.g, dot.color.b, dot.color.a = 0.0, 1.0, 0.0, 1.0
                    dot.pose.position.x, dot.pose.position.y, dot.pose.position.z = (
                        float(p[0]), float(p[1]), float(p[2]),
                    )
                    dot.pose.orientation.w = 1.0
                    array.markers.append(dot)

                text = Marker()
                text.header.frame_id = link
                text.header.stamp = stamp
                text.ns = "rsdf_surfaces_labels"
                text.id = i
                text.type = Marker.TEXT_VIEW_FACING
                text.action = Marker.ADD
                text.scale.z = 0.03
                text.color.r, text.color.g, text.color.b, text.color.a = 1.0, 1.0, 1.0, 1.0
                text.pose.position.x, text.pose.position.y, text.pose.position.z = (
                    float(center[0]), float(center[1]), float(center[2]),
                )
                text.pose.orientation.w = 1.0
                text.text = name
                array.markers.append(text)
                continue

            fill = Marker()
            fill.header.frame_id = link
            fill.header.stamp = stamp
            fill.ns = "rsdf_surfaces"
            fill.id = 2 * i
            fill.type = Marker.TRIANGLE_LIST
            fill.action = Marker.ADD
            fill.scale.x = fill.scale.y = fill.scale.z = 1.0
            fill.color.r, fill.color.g, fill.color.b, fill.color.a = 0.2, 0.8, 0.2, 0.4
            fill.pose.orientation.w = 1.0
            for j in range(len(points_3d)):
                p0, p1 = points_3d[j], points_3d[(j + 1) % len(points_3d)]
                for p in (center, p0, p1):
                    fill.points.append(Point(x=float(p[0]), y=float(p[1]), z=float(p[2])))
            array.markers.append(fill)

            outline = Marker()
            outline.header.frame_id = link
            outline.header.stamp = stamp
            outline.ns = "rsdf_surfaces"
            outline.id = 2 * i + 1
            outline.type = Marker.LINE_STRIP
            outline.action = Marker.ADD
            outline.scale.x = 0.005
            outline.color.r, outline.color.g, outline.color.b, outline.color.a = 0.0, 1.0, 0.0, 1.0
            outline.pose.orientation.w = 1.0
            for p in points_3d + [points_3d[0]]:
                outline.points.append(Point(x=float(p[0]), y=float(p[1]), z=float(p[2])))
            array.markers.append(outline)

            text = Marker()
            text.header.frame_id = link
            text.header.stamp = stamp
            text.ns = "rsdf_surfaces_labels"
            text.id = i
            text.type = Marker.TEXT_VIEW_FACING
            text.action = Marker.ADD
            text.scale.z = 0.03
            text.color.r, text.color.g, text.color.b, text.color.a = 1.0, 1.0, 1.0, 1.0
            text.pose.position.x, text.pose.position.y, text.pose.position.z = (
                float(center[0]), float(center[1]), float(center[2]),
            )
            text.pose.orientation.w = 1.0
            text.text = name
            array.markers.append(text)

        self.pub.publish(array)


def main():
    if len(sys.argv) < 2:
        print("Usage: visualize_rsdf_surfaces.py <path-to-rsdf-file>")
        sys.exit(1)
    rclpy.init()
    node = RsdfMarkerPublisher(sys.argv[1])
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
