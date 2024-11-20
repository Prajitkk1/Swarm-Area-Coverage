# Swarm-Area-Coverage
# multi-UAV Path Planning with Nearest Neighbor Search

This project implements an efficient **robot path planning system** for covering polygonal areas while avoiding no-go zones. Using a nearest neighbor search algorithm, the system calculates optimal paths for robots to traverse valid cells, starting from a user-defined initial position. It provides both visualization and performance metrics, making it ideal for applications like UAV or AGV path planning in agriculture, environmental monitoring, or industrial settings.

---

## Features

### 🚀 Core Functionalities
- **Boundary and No-Go Zone Management**:
  - Handles arbitrary polygonal boundaries and obstacles (no-go zones).
  - Automatically excludes invalid cells from the path.

- **Partitioning**:
  - Divides the area into user-defined partitions, ensuring partition boundaries align with cell edges.

- **Path Planning**:
  - Uses a **nearest neighbor algorithm** with KD-tree optimization to compute the shortest path through valid cells.
  - Robots start from a user-specified initial position.

- **Performance Metrics**:
  - Calculates the total travel distance and time required for robots in each partition.

### 🎨 Visualization
- Displays:
  - Polygonal boundary and no-go zones.
  - Valid cells (in light grey).
  - Robots' paths (in unique colors for each partition).
- Provides a clear visual representation of coverage and obstacles.

---

## Usage
- Clone the repository
- Run NaiveScanningAreaCoverage.py
- You can modify the starting points, boundaries and no-go zones as desired.
- The current locations are obtained from the work Guastella et al. **Complete coverage path planning for aerial vehicle flocks deployed in outdoor environments.** and Collins et al. ** Scalable Coverage Path Planning of Multi-Robot Teams for Monitoring Non-Convex Areas**
- Required Libraries: numpy, matplotlib, shapely, scikit-learn
- Cell size indicates the FoV of the UAV or the the square size in which the area should be discretized. 

## Citing

If you use *Swarm-Area-Coverage* in your research, please cite it using the following Bibtex entry:

```bibtex
@software{prajit_2024_14190023,
  author       = {Prajit, Krisshna Kumar},
  title        = {Prajitkk1/Swarm-Area-Coverage: v1.0},
  month        = nov,
  year         = 2024,
  publisher    = {Zenodo},
  version      = {v1.0},
  doi          = {10.5281/zenodo.14190023},
  url          = {https://doi.org/10.5281/zenodo.14190023}
}
