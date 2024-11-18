# Robots painting music

## Introduction
This project integrates robotics with music and painting to create a system where robots paint based on musical features extracted from an audio signal. The system uses a swarm of robots leaving paint on the trail to visualize music in real-time, creating dynamic and visually appealing patterns.

## Set up

### Environment and Denpendencies
- Ubuntu 22.04.5 LTS
- python 3.10
- numpy 1.25.2
- scipy
- matplotlib
- essentia


## User Guide

### Project Structure
- **`music.py`**: Contains functions for music feature extraction, such as chord and tempo detection.
- **`pipeline.py`**: Maps musical features to robot control parameters.
- **`swarm.py`**: Conrtol the swarm of robots and their collective behavior.
- **`robot.py`**: Defines the behavior and control of individual robots.
- **`voronoi.py`**: Computes Voronoi cells for the robots' positions.
- **`simulator.py`**: Visualizes the robots and their movements.
- **`consts.py`**: Contains constant values used throughout the project.
- **`utils.py`**: Utility functions and classes.


### Running the Project
1. **Start the simulator**:
    ```bash
      python main.py
    ```

2. **Interacting with the simulator**:
  - The simulator will visualize the robots and their movements.
  - You can click on the simulation area to set the center for the robots' movements 
  - You can move the slider to control the parameters of robots

## API document
#### `Swarm.color_coverage_control(self, density_functions=None)`

Compute the voronoi cell, centroid and area of all prime colors for each robot in the swarm, based on the provided density functions.

**Parameters**:
- `density_functions` (list, optional): A list of density functions to be used for controlling the color coverage. If not provided, the existing density functions will be used.

**Returns**:
- `vor_robots` (dict): A dictionary where the keys are robot indices and the values are dictionaries with color keys (CYAN, MAGENTA, YELLOW) and their corresponding Voronoi cell information (centroid, cell, area).
- `vor_prime` (dict): A dictionary with color keys (CYAN, MAGENTA, YELLOW) and their corresponding prime Voronoi cell information (centroid, cell, area).


#### `Swarm.coverage_control(self, density_functions=None)`
Calculates the Voronoi cells, centroids, and areas for a given set of robots based on a density function.

**Parameters**:

- `robots` (list): A list of robot objects for which the Voronoi cells are to be calculated.
- `density_function` (function): A density function used to calculate the Voronoi centroids and areas.

**Returns**:
- `vor_centroid` (ndarray): An array of Voronoi centroids for each robot.
- `vor_cell` (ndarray): An array of Voronoi cells for each robot.
- `vor_area` (ndarray): An array of Voronoi cell areas for each robot.

