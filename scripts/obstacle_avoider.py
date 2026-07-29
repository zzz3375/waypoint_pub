"""Obstacle avoidance module — body-frame LaserScan → movement-direction guard + RViz viz."""

import numpy as np
import rospy
from sensor_msgs.msg import LaserScan, PointCloud2, PointField
import sensor_msgs.point_cloud2 as pc2


class ObstacleAvoider:
    """Consumes LaserScan messages (body-frame, 360°) and answers two questions:

       1. should_hold(pos, tgt, yaw) → bool     — block movement?
       2. publish_obstacle_cloud(pos, yaw)       — red points in RViz
    """

    def __init__(self, safe_distance=5.0, sector_half_deg=30,
                 cloud_topic="/obstacle_cloud", cloud_frame="body"):
        self.safe_dist   = safe_distance
        self.sector_half = np.deg2rad(sector_half_deg)

        # ---- scan cache ----
        self._data  = None          # numpy float64 1-D  (full 360°)
        self._amin  = 0.0           # angle_min
        self._ainc  = 1.0           # angle_increment

        # ---- point-cloud publisher (throttled to ~10 Hz) ----
        self._cloud_pub = rospy.Publisher(cloud_topic, PointCloud2, queue_size=1)
        self._cloud_frame = cloud_frame
        self._last_cloud_t = 0.0

    # ==================================================================
    #  public API
    # ==================================================================

    def feed_scan(self, msg):
        """Call from a rospy.Subscriber(LaserScan) callback once per message."""
        n = len(msg.ranges)
        if n == 0:
            return
        self._data = np.array(msg.ranges, dtype=np.float64)
        self._amin = msg.angle_min
        self._ainc = msg.angle_increment

    def should_hold(self, pos_world, tgt_world, yaw_world):
        """Return True when an obstacle lies inside the movement-direction sector.

        Args:
            pos_world:  (3,) float — current [x, y, z]          (world / map)
            tgt_world:  (3,) float — target  [x, y, z]          (world / map)
            yaw_world:  float      — current yaw                  (rad, world)
        """
        if self._data is None:
            return False

        # ---- movement direction (world → body) ----
        dx, dy = tgt_world[0] - pos_world[0], tgt_world[1] - pos_world[1]
        c, s = np.cos(-yaw_world), np.sin(-yaw_world)
        bx = dx * c - dy * s
        by = dx * s + dy * c
        angle_body = np.arctan2(by, bx)

        return self._min_range(angle_body, self.sector_half) < self.safe_dist

    def publish_obstacle_cloud(self, pos_world, yaw_world):
        """Publish a PointCloud2 of all scan points whose range < safe_dist.

        Throttled internally to ≈10 Hz to avoid flooding RViz.
        Points are published in body-frame directly (no world/map transform).
        """
        now = rospy.Time.now().to_sec()
        if now - self._last_cloud_t < 0.1 or self._data is None:
            return
        self._last_cloud_t = now

        # ---- body-frame angles for EVERY bin ----
        n = len(self._data)
        angles = self._amin + np.arange(n, dtype=np.float64) * self._ainc

        # ---- mask: within safe distance ----
        mask = np.isfinite(self._data) & (self._data < self.safe_dist)
        if not np.any(mask):
            return

        r = self._data[mask]            # valid ranges
        a = angles[mask]                # matching body-frame angles

        # ---- body-frame coordinates (no world/map transform) ----
        x_b = r * np.cos(a)
        y_b = r * np.sin(a)
        z_b = np.zeros_like(x_b)

        pts = np.column_stack((x_b, y_b, z_b))

        header = rospy.Header(frame_id=self._cloud_frame, stamp=rospy.Time.now())
        fields = [PointField('x', 0, PointField.FLOAT32, 1),
                  PointField('y', 4, PointField.FLOAT32, 1),
                  PointField('z', 8, PointField.FLOAT32, 1)]
        cloud = pc2.create_cloud(header, fields, pts.astype(np.float32))
        self._cloud_pub.publish(cloud)

    # ==================================================================
    #  internal
    # ==================================================================

    def _min_range(self, angle_body, half_angle):
        """Vectorised: min finite range inside [angle_body ± half_angle]."""
        n = len(self._data)
        hc = int(half_angle / self._ainc)
        ctr = int(round((angle_body - self._amin) / self._ainc))
        idx = (np.arange(-hc, hc + 1, dtype=int) + ctr) % n
        vals = self._data[idx]
        return np.min(vals[np.isfinite(vals)], initial=np.inf)
