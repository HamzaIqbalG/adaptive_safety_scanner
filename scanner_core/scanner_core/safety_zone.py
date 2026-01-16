import numpy as np
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
from std_msgs.msg import ColorRGBA

class SafetyZone:
    def __init__(self, name, zone_id, points, color_tuple):
        """
        points: List of (x, y) tuples defining the polygon
        color_tuple: (r, g, b, a)
        """
        self.name = name
        self.zone_id = zone_id
        self.points = points # List of (x,y)
        self.color = color_tuple

    def get_marker(self, frame_id="laser"):
        """Returns a LINE_STRIP marker to visualize the zone boundary"""
        marker = Marker()
        marker.header.frame_id = frame_id
        marker.ns = "zones"
        marker.id = self.zone_id
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.scale.x = 0.05  # Line width

        # Set Color
        marker.color.r = self.color[0]
        marker.color.g = self.color[1]
        marker.color.b = self.color[2]
        marker.color.a = self.color[3]

        # Add points (close the loop by adding the first point at the end)
        for x, y in self.points:
            p = Point()
            p.x = float(x)
            p.y = float(y)
            p.z = -0.1 # Draw slightly below the floor (z=0) so it doesn't overlap objects
            marker.points.append(p)

        # Close the loop
        p_first = Point()
        p_first.x = float(self.points[0][0])
        p_first.y = float(self.points[0][1])
        p_first.z = -0.1
        marker.points.append(p_first)

        # Lifetime (0 = forever)
        marker.lifetime.sec = 0

        return marker

    def contains(self, x, y):
        """
        Ray Casting Algorithm to check if point (x,y) is inside polygon
        """
        n = len(self.points)
        inside = False
        p1x, p1y = self.points[0]
        for i in range(n + 1):
            p2x, p2y = self.points[i % n]
            if y > min(p1y, p2y):
                if y <= max(p1y, p2y):
                    if x <= max(p1x, p2x):
                        if p1y != p2y:
                            xinters = (y - p1y) * (p2x - p1x) / (p2y - p1y) + p1x
                        if p1x == p2x or x <= xinters:
                            inside = not inside
            p1x, p1y = p2x, p2y
        return inside