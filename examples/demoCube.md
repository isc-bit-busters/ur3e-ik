# UR3e Inverse Kinematics and Path Planning

## Overview

This module computes inverse kinematics for the **UR3e robotic arm**, generates movement paths, and creates trajectory files for demo execution of a Cube.

## Available Methods

### 1. Inverse Kinematics Computation

#### `compute_inverse_kinematics(position, roll, pitch, yaw)`

Converts a target position and orientation into joint angles.

**Parameters:**

- `position` *(tuple)*: (x, y, z) in meters.
- `roll, pitch, yaw` *(float)*: Rotation angles (degrees).

**Returns:**

- Joint angle solutions (or None if no solution).

**Usage:**

```python
angles = compute_inverse_kinematics((0.2, 0.2, 0.2), roll=0, pitch=90, yaw=0)
```

### 2. Path Generation

#### `generate_path_square(origin, direction, width, length, step, depth)`

Creates a zigzag path for robotic arm movement.&#x20;

**Parameters:**

- `origin` *(tuple)*: (x, y, z) start position.
- `direction` *(tuple)*: Movement direction.
- `width, length, step, depth` *(float)*: Path dimensions.

**Returns:**

- List of (x, y, z) positions.

**Usage:**

```python
path = generate_path_square((0.2, 0.2, 0.2), (1, 0, 0), 0.15, 0.15, 0.05, 0.05)
```

### 3. Orientation Computation

#### `compute_orientation_towards_target(origin, target, offset=(0.0, 0.0, 0.0))`

Computes roll, pitch, yaw angles to face a target.

**Parameters:**

- `origin, target` *(tuple)*: (x, y, z) positions.
- `offset` *(tuple)*: Adjustments (degrees).

**Returns:**

- Roll, pitch, yaw angles.

**Usage:**

```python
roll, pitch, yaw = compute_orientation_towards_target((0.2, 0.2, 0.2), (0.2, 0.2, 0.0))
```

### 4. Transforming Coordinates to Joint Angles

#### `transform_coordinates_to_joint_angles(coordinates, orientation=(None, None, None))`

Converts Cartesian coordinates into joint angles.

**Parameters:**

- `coordinates` *(list of tuples)*: (x, y, z) positions.
- `orientation` *(tuple)*: Roll, pitch, yaw (or auto-compute towards target position and z=0).

**Returns:**

- List of joint angles.

**Usage:**

```python
joint_angles = transform_coordinates_to_joint_angles(path, (0, 179.942, 0))
```

### 5. Trajectory File Generation

#### `generate_trajectory_file(data, filename="trajectory.json")`

Saves joint angles to a JSON file.

**Parameters:**

- `data` *(list of lists)*: Joint angles.
- `filename` *(str)*: Output file.

**Usage:**

```python
generate_trajectory_file(joint_angles, "robot_trajectory.json")
```

## Example Pipeline

This is a complete example showing how to generate a movement path, compute joint angles, and save a trajectory file:

```python
# Step 1: Generate a movement path
path = generate_path_square(
    origin=(0.2, 0.2, 0.2),  # Starting point of the movement
    direction=(1, 0, 0),  # Movement progresses along the X-axis
    width=0.15,  # Width of the area to cover
    length=0.15,  # Length of the area to cover
    step=0.05,  # Step size between zigzag movements
    depth=0.05  # Depth adjustment at the end of each row
)

# Step 2: Compute joint angles for the path
joint_angles = transform_coordinates_to_joint_angles(
    path, 
    orientation=(0, 179.942, 0)  # Roll, pitch, yaw angles in degrees
)

# Step 3: Save the computed trajectory to a JSON file
generate_trajectory_file(joint_angles, filename="square_trajectory.json")
```

## Additional Notes

### **Handling Orientation Issues (0° and 180°)**

- **Problem**: `0°` or `180°` orientation can cause no valid solutions.
- **Solution**: Use slight offsets (e.g., `179.942°`) to prevent singularities.

