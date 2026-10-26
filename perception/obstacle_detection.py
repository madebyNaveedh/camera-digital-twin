"""
Obstacle detection and feature extraction from 2D spatial point clouds.
"""

from dataclasses import dataclass, field
from typing import List, Optional
import math
import numpy as np


@dataclass
class ObstacleCandidate:
    id: int
    centroid: np.ndarray          # [x, y] in Room coordinates
    radius: float                 # Estimated bounding cylinder radius (m)
    point_count: int
    confidence: float             # Value in [0.0, 1.0]
    points: np.ndarray            # Points belonging to cluster
    bbox_extent: np.ndarray       # [width, length]


class ObstacleDetector:
    def __init__(self,
                 distance_threshold: float = 0.28,
                 min_cluster_size: int = 4,
                 max_cluster_size: int = 250,
                 min_obstacle_radius: float = 0.05,
                 max_obstacle_radius: float = 0.75):
        """
        Args:
            distance_threshold: Maximum Euclidean distance between consecutive or nearby points to merge into cluster.
            min_cluster_size: Minimum number of points required for a cluster.
            max_cluster_size: Clusters larger than this are ignored (likely walls).
            min_obstacle_radius: Minimum allowable radius for an obstacle (m).
            max_obstacle_radius: Maximum allowable radius for an obstacle (m).
        """
        self.distance_threshold = distance_threshold
        self.min_cluster_size = min_cluster_size
        self.max_cluster_size = max_cluster_size
        self.min_obstacle_radius = min_obstacle_radius
        self.max_obstacle_radius = max_obstacle_radius

    def cluster_points(self, points: np.ndarray) -> List[np.ndarray]:
        """
        Fast 2D Euclidean clustering of point cloud.
        Groups points where distance between neighbors is <= distance_threshold.
        """
        if points.size == 0 or len(points) < self.min_cluster_size:
            return []

        n = len(points)
        visited = np.zeros(n, dtype=bool)
        clusters = []

        # For sequential LiDAR scan points, points close in index are often close in space.
        # We perform a breadth-first search / region-growing clustering.
        for i in range(n):
            if visited[i]:
                continue

            current_cluster = [i]
            visited[i] = True
            queue = [i]

            while queue:
                curr_idx = queue.pop(0)
                curr_pt = points[curr_idx]

                # Find unvisited neighbors within distance_threshold
                # Vectorized distance to unvisited points
                unvisited_indices = np.where(~visited)[0]
                if len(unvisited_indices) == 0:
                    break

                diffs = points[unvisited_indices] - curr_pt
                dists_sq = np.sum(diffs * diffs, axis=1)
                neighbor_mask = dists_sq <= (self.distance_threshold ** 2)

                matching_indices = unvisited_indices[neighbor_mask]
                for n_idx in matching_indices:
                    visited[n_idx] = True
                    queue.append(n_idx)
                    current_cluster.append(n_idx)

            if self.min_cluster_size <= len(current_cluster) <= self.max_cluster_size:
                clusters.append(points[current_cluster])

        return clusters

    def detect_obstacles(self, points_room_2d: np.ndarray) -> List[ObstacleCandidate]:
        """
        Detect obstacle candidates from 2D points in the room frame.
        """
        clusters = self.cluster_points(points_room_2d)
        candidates = []

        for cid, cluster_pts in enumerate(clusters):
            # Raw centroid of visible surface arc
            raw_centroid = np.mean(cluster_pts, axis=0)
            
            # Radii of points relative to centroid
            diffs = cluster_pts - raw_centroid
            dists = np.linalg.norm(diffs, axis=1)
            radius = float(np.max(dists)) if len(dists) > 0 else self.min_obstacle_radius
            radius = max(self.min_obstacle_radius, min(radius, self.max_obstacle_radius))

            # Compensate for front-facing surface occlusion:
            # Shift centroid away from LiDAR along line-of-sight to estimate object center
            norm_raw = np.linalg.norm(raw_centroid)
            if norm_raw > 1e-3:
                dir_los = raw_centroid / norm_raw
                # Arc centroid is offset from true center by approximately (2/pi) * radius
                center_offset = (2.0 / math.pi) * radius
                centroid = raw_centroid + dir_los * center_offset
            else:
                centroid = raw_centroid

            # Bounding box extents
            pt_min = np.min(cluster_pts, axis=0)
            pt_max = np.max(cluster_pts, axis=0)
            bbox_extent = pt_max - pt_min

            # Reject clusters that are overly elongated (like wall segments)
            max_extent = max(bbox_extent[0], bbox_extent[1])
            if max_extent > (self.max_obstacle_radius * 2.5):
                continue

            # Confidence based on point count and compactness
            count = len(cluster_pts)
            confidence = min(1.0, count / 15.0)

            candidate = ObstacleCandidate(
                id=cid,
                centroid=centroid,
                radius=radius,
                point_count=count,
                confidence=confidence,
                points=cluster_pts,
                bbox_extent=bbox_extent
            )
            candidates.append(candidate)

        return candidates
