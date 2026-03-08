# Robot Cleaner

A complete production-quality C++17 project that solves a robotics coding challenge involving path metric computation, polygon area cleaning calculation, and graphical visualization.

## Features
- **Path Length Calculation**: Computes the total traversal distance using Euclidean distance between waypoints.
- **Curvature Computation**: Approximates discrete points curvature using the **circumcircle method**.
- **Velocity Profiling**: Assigns speeds segment-by-segment based on curvature limits (`vmax`, `vmin`, `k_crit`, `k_max`).
- **Cleaned Area Computation**: Uses Boost.Geometry to calculate the total swept area composed of sweeping a cleaning line across path segments. Overlaps are inherently discounted through boolean polygon union functionality.
- **Visualization**: An automated Qt6 QGraphicsView rendering engine mapping path points, robot footprint geometry, cleaning gadgets, and color-coding path segments corresponding to their curvature levels (Green=Straight, Red=Sharp curves).

## Mathematical Formulas Used
### Curvature (Circumcircle Method)
Given 3 path points $P_{i-1}, P_{i}, P_{i+1}$, they form a triangle with sides $a, b, c$ and Area $A$ (via Heron's formula).
Curvature $\kappa$ is computed as:
$$ \kappa = \frac{4A}{abc} $$
If the points are collinear ($A \approx 0$), curvature is $0$.

### Velocity Model
$$ 
v = \begin{cases} 
    v_{max} & \text{if } \kappa \leq k_{crit} \\
    v_{max} \times \frac{k_{crit}}{\kappa} & \text{if } k_{crit} < \kappa < k_{max} \\
    v_{min} & \text{if } \kappa \geq k_{max} 
\end{cases}
$$

### Traversal Time
Overall time is the sum of times on all discrete straight-line segments.
$$ T = \sum \frac{dist(P_{i-1}, P_i)}{v_{i}} $$
where $v_i$ is computed from the average of curvatures at the segment endpoints.

## Dependencies
- C++17 Compatible compiler (GCC, Clang)
- CMake 3.16+
- **Boost** (Requires local installation, specifically `boost::geometry`)
- **Qt6** (Requires local installation)
- **Catch2** (Automatically fetched via CMake FetchContent)
- **nlohmann/json** (Automatically fetched via CMake FetchContent)

## Build Instructions

```bash
mkdir build
cd build
cmake ..
make
```

## Running the Verification Tests
To execute all Catch2 unit test boundaries:
```bash
cd build
ctest --output-on-failure
```

## Example Run
Execute the main application providing an example JSON configurations file. The default configuration uses `data/short.json`.

```bash
./robot_cleaner ../data/short.json
```

**Console Output Check:**
- Prints calculated total path length (meters).
- Prints total area spanned by the cleaning line (square meters).
- Prints traversal time (seconds).

**Visualization Explanation:**
A Qt6 window will automatically appear framing the operational area bounding box:
- **Grey outlines** are robot shapes sampled iteratively.
- **Red lines** designate the sweeping line attached to the robot.
- **Blue shaded polygons (transparent)** encompass the net swept volume via union mappings.
- **Central Trajectory** indicates movement, gradient-colored by severity of derived curvature.

## Design Decisions
1. **Header/Source Separation**: Adherence to standard C++ practices allows cleaner test linkage.
2. **Boolean Operations**: `boost::geometry` provides mathematically robust intersection and union utilities, avoiding manual edge overlap corner cases natively.
3. **Visualization scaling coordinate inversion**: Because standard math uses 'y-up' and Qt natively assumes 'y-down' for canvas elements, an explicit matrix affine transformation `scale(1, -1)` preserves input data layout perfectly on screen natively without rewriting transformation logic components.
