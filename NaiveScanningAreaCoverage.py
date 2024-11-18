# -*- coding: utf-8 -*-
"""
Created on Mon Nov 18 13:23:01 2024

@author: Prajit
"""

import numpy as np
import matplotlib.pyplot as plt
from shapely.geometry import Polygon as ShapelyPolygon, box
from shapely.ops import unary_union
from sklearn.neighbors import NearestNeighbors

class RobotPathPlanner:
    def __init__(self, boundary_polygon, no_go_zones, cell_size, speed, start_x, start_y, partition_count):
        """
        Initialize the RobotPathPlanner.
        
        Parameters:
        - boundary_polygon: ShapelyPolygon representing the scanning area.
        - no_go_zones: List of ShapelyPolygon objects representing no-go zones.
        - cell_size: Size of each cell (e.g., 2 meters).
        - speed: Robot speed (meters per second).
        - start_x, start_y: Starting position of the robot.
        - partition_count: Number of partitions for the area.
        """
        self.boundary_polygon = boundary_polygon
        self.no_go_zones = no_go_zones
        self.combined_no_go_zone = unary_union(no_go_zones)
        self.cell_size = cell_size
        self.speed = speed
        self.start_x = start_x
        self.start_y = start_y
        self.partition_count = partition_count
        self.partition_lines = self._adjust_partition_lines()
        self.robot_paths = []
        self.times_per_partition = []
        self.distances_per_partition = []

    def _adjust_partition_lines(self):
        """
        Adjust partition lines to align with cell boundaries.
        """
        min_x, _, max_x, _ = self.boundary_polygon.bounds
        partition_lines = np.linspace(min_x, max_x, self.partition_count + 1)
        adjusted_lines = [min_x]
        for line in partition_lines[1:]:
            adjusted_line = min_x + np.round((line - min_x) / self.cell_size) * self.cell_size
            if adjusted_line not in adjusted_lines and adjusted_line < max_x:
                adjusted_lines.append(adjusted_line)
        adjusted_lines.append(max_x)
        return adjusted_lines

    def _calculate_nearest_neighbor_path(self, valid_area):
        """
        Calculate robot path using the nearest neighbor algorithm for a given area.
        """
        min_x, min_y, max_x, max_y = valid_area.bounds
        valid_cells = []

        for x in np.arange(min_x, max_x, self.cell_size):
            for y in np.arange(min_y, max_y, self.cell_size):
                cell = box(x, y, x + self.cell_size, y + self.cell_size)
                if valid_area.contains(cell.centroid):
                    valid_cells.append((x + self.cell_size / 2, y + self.cell_size / 2))

        if not valid_cells:
            return 0, [(self.start_x, self.start_y)]  # No valid cells to visit

        # Include the robot's starting position
        task_list = valid_cells[:]
        task_list.append((self.start_x, self.start_y))

        current_position = (self.start_x, self.start_y)
        path = [current_position]
        total_distance = 0

        while task_list:
            nbrs = NearestNeighbors(n_neighbors=1, algorithm='kd_tree').fit(task_list)
            distances, indices = nbrs.kneighbors([current_position])
            next_position = task_list[indices[0][0]]

            total_distance += distances[0][0]
            path.append(next_position)

            task_list.remove(next_position)
            current_position = next_position

        return total_distance, path

    def calculate_paths(self):
        """
        Calculate paths for each partition using the nearest neighbor algorithm.
        """
        for i in range(len(self.partition_lines) - 1):
            strip_min_x = self.partition_lines[i]
            strip_max_x = self.partition_lines[i + 1]
            partition_area = box(strip_min_x, self.boundary_polygon.bounds[1], strip_max_x, self.boundary_polygon.bounds[3])
            valid_area = self.boundary_polygon.difference(self.combined_no_go_zone).intersection(partition_area)

            distance, path = self._calculate_nearest_neighbor_path(valid_area)
            self.distances_per_partition.append(distance)
            self.times_per_partition.append(distance / self.speed)  # Time in seconds
            self.robot_paths.append(path)

    def plot_results(self):
        """
        Plot the robot paths and cells in the scanning area.
        """
        fig, ax = plt.subplots(figsize=(12, 8))

        # Plot boundary polygon
        x, y = self.boundary_polygon.exterior.xy
        ax.plot(x, y, 'r-', label='Boundary Polygon')

        # Plot no-go zones
        for idx, zone in enumerate(self.no_go_zones):
            x, y = zone.exterior.xy
            if idx == 0:
                ax.fill(x, y, 'red', alpha=0.3, label='No-go Zone')
            else:
                ax.fill(x, y, 'red', alpha=0.3)

        # Plot robot paths
        colormap = plt.cm.get_cmap('viridis', len(self.robot_paths))
        for idx, path in enumerate(self.robot_paths):
            if path:
                x, y = zip(*path)
                ax.plot(x, y, color=colormap(idx), alpha=0.8, label=f'UAV Path {idx + 1}')

        # Scatter valid cells as light grey squares
        x_coords = np.arange(self.boundary_polygon.bounds[0], self.boundary_polygon.bounds[2], self.cell_size)
        y_coords = np.arange(self.boundary_polygon.bounds[1], self.boundary_polygon.bounds[3], self.cell_size)
        for x in x_coords:
            for y in y_coords:
                cell = box(x, y, x + self.cell_size, y + self.cell_size)
                if self.boundary_polygon.contains(cell.centroid) and not self.combined_no_go_zone.contains(cell.centroid):
                    rect = plt.Rectangle((x, y), self.cell_size, self.cell_size, color='lightgrey', alpha=0.3)
                    ax.add_patch(rect)

        # Finalize plot
        ax.set_xlabel('X (m)')
        ax.set_ylabel('Y (m)')
        ax.legend()
        ax.grid(True)
        ax.axis('equal')
        plt.show()

    def print_results(self):
        """
        Print the time and distance for each partition.
        """
        for i, (time, distance) in enumerate(zip(self.times_per_partition, self.distances_per_partition), start=1):
            print(f"Partition {i} - Time required: {time:.2f} seconds, Distance covered: {distance:.2f} units")


# Example usage:
if __name__ == "__main__":
    boundary_polygon = ShapelyPolygon([
        (18, 95), (105, 81), (93, 0), (43, 2), (20, 16), (0, 19), (0, 76), (18, 95)
    ])
    no_go_zones = [
        ShapelyPolygon([(40, 29), (46, 29), (43, 11), (37, 11)]),
        ShapelyPolygon([(23, 72), (29, 71), (34, 61), (23, 62)]),
        ShapelyPolygon([(50, 89), (65, 77), (64, 67), (54, 68), (53, 58), (46, 59)]),
        ShapelyPolygon([(72, 76), (79, 75), (84, 74), (85, 64), (80, 55), (76, 55), (69, 56)])
    ]

    planner = RobotPathPlanner(boundary_polygon, no_go_zones, cell_size=2, speed=4, start_x=55, start_y=44, partition_count=9)
    planner.calculate_paths()
    planner.plot_results()
    planner.print_results()
