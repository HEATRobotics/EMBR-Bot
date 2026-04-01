
#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2
import numpy as np
from sklearn.cluster import DBSCAN
import sys

class PointCloudReader(Node):
    def __init__(self):
        super().__init__('pointcloud_reader')
        self.create_subscription(PointCloud2, '/scan/points', self.pc_callback, 10)

    def pc_callback(self, msg):
        points = []
        point_counter = 0
        detection_range = 4.0
        height_threshold = -0.1

        for p in pc2.read_points(msg, field_names=('x','y','z'), skip_nans=True):
            x, y, z = p

            # Remove ground
            if z < height_threshold:
                continue

            if z > 100:
                continue

            if z > height_threshold:
                point_counter += 1

            # Horizontal distance filter (ignore z)
            horizontal_dist = math.sqrt(x*x + y*y)
            if horizontal_dist > detection_range:
                continue

            points.append([x, y, z])

        if len(points) == 0:
            return

        points = np.array(points)

        clustering = DBSCAN(eps=0.2, min_samples=10).fit(points)
        labels = clustering.labels_

        unique_clusters = set(labels)
        unique_clusters.discard(-1)  # remove noise label

        cluster_summaries = []
        for cluster_id in sorted(unique_clusters):
            cluster_points = points[labels == cluster_id]

            center = np.mean(cluster_points, axis=0)
            range_m = math.sqrt(center[0] * center[0] + center[1] * center[1])

            angles = np.arctan2(cluster_points[:, 1], cluster_points[:, 0])
            angle_span = float(np.max(angles) - np.min(angles))
            apparent_width = 2.0 * range_m * math.sin(angle_span / 2.0)

            min_z = float(np.min(cluster_points[:, 2]))
            max_z = float(np.max(cluster_points[:, 2]))
            height_m = max_z - min_z

            cluster_summaries.append(
                f"obj{cluster_id}: range={range_m:.2f}m width={apparent_width:.2f}m height={height_m:.2f}m"
            )

        cluster_summary_text = " | ".join(cluster_summaries) if cluster_summaries else "no clustered objects"

        sys.stdout.write(
            f"\rDetected {len(unique_clusters)} objects, Total points above threshold: {point_counter}, "
            f"Detection range = {detection_range}, Height threshold = {height_threshold} | {cluster_summary_text}"
        )
        sys.stdout.flush()


def main(args=None):
    rclpy.init(args=args)
    node = PointCloudReader()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

