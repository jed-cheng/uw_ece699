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
      python main.py <audio>
    ```

2. **Interacting with the simulator**:
  - The simulator will visualize the robots and their movements.
  - You can click on the simulation area to set the center for the robots' movements 
  - You can move the slider to control the parameters of robots

### Usage

```bash
python [main.py] audio [-h] [--output OUTPUT] [--robot ROBOT] [--color COLOR] [--trail TRAIL] [--l L] 
```
#### Required Arguments
- `<audio_file>`: Path to the audio file to be used for extracting musical features.
#### Optional Arguments

- `--output <output_file>`: Path to the output file where the simulation result will be saved. Default is output.png.
- `--robot <num_robot>`: Number of robots to be used in the simulation. Possible values are 6, 9, and 12. Default is 6.
- `--color <num_color>`: Number of colors each robot is equipped with. Possible values are 1, 2, and 3. Default is 3.
- `--trail <trail_width>`: Width of the trail left by the robots. Possible values are 10, 15, and 20. Default is 15.
- `--l <L_value>`: The L parameter for the robots. Possible values are 1, 3, and 5. Default is 1.

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



#### `Robot.coverage_control_color(self, vor_robot, cyan_density, magenta_density, yellow_density)`

Calculates the color coverage control for a robot based on the Voronoi cell and density functions.

**Parameters**:
- `vor_robot` (dict): A dictionary where the keys are color values (CYAN, MAGENTA, YELLOW) and the values are tuples containing the Voronoi cell information (centroid, cell, area) for the robot.
- `cyan_density` (DensityFunction): The density function for the cyan color.
- `magenta_density` (DensityFunction): The density function for the magenta color.
- `yellow_density` (DensityFunction): The density function for the yellow color.

**Returns**:
- `color` (ndarray): An array representing the color coverage control for the robot.

**Description**:
This function calculates the color coverage control for a robot by determining the proportion of each color's area within the robot's Voronoi cell and adjusting the color based on the density functions.

#### `Robot.coverage_control(self, vor_robot, delta=None)`

Calculates the control input for a robot to move towards the centroid of its Voronoi cell.

**Parameters**:
- `vor_robot` (dict): A dictionary where the keys are color values (CYAN, MAGENTA, YELLOW) and the values are tuples containing the Voronoi cell information (centroid, cell, area) for the robot.
- `delta` (float, optional): The time delta for the movement. Default is `None`.

**Returns**:
- `vw` (ndarray): An array representing the linear and angular velocities for the robot.

**Description**:
This function calculates the control input for a robot to move towards the centroid of its Voronoi cell. It computes the control input based on the area-weighted centroid of the Voronoi cell and adjusts the robot's speed accordingly.

#### `Voronoi.voronoi_centroids(vor_cell, density_function)`

Calculates the centroid and area of a Voronoi cell based on a given density function.

**Parameters**:
- `vor_cell` (ndarray): An array of points representing the vertices of the Voronoi cell.
- `density_function` (function): A density function used to calculate the weighted centroid and area of the Voronoi cell.

**Returns**:
- `centroid` (ndarray): An array representing the coordinates of the centroid of the Voronoi cell.
- `area` (float): The area of the Voronoi cell.

**Description**:
This function calculates the centroid and area of a Voronoi cell by integrating the given density function over the cell's area. The centroid is weighted by the density function, providing a more accurate representation of the cell's center of mass.

#### `EmotionPipeline.predict_emotion(self, chord)`

Predicts the emotions associated with a given chord.

**Parameters**:
- `chord` (str): The chord symbol for which to predict the emotions.

**Returns**:
- `emotions` (list): A list of predicted emotions associated with the given chord.

**Description**:
This function predicts the emotions associated with a given chord based on predefined mappings between chords and emotions. The function uses the chord symbol to look up the corresponding emotions and returns them as a list.


#### `ColorPipeline.predict_colors(self, emotions)`

Predicts the colors associated with a given list of emotions.

**Parameters**:
- `emotions` (list): A list of emotions for which to predict the colors.

**Returns**:
- `colors` (list): A list of predicted colors associated with the given emotions.

**Description**:
This function predicts the colors associated with a given list of emotions based on predefined mappings between emotions and colors. The function uses the list of emotions to look up the corresponding colors and returns them as a list.

#### `CenterPipeline.predict_center(self, chord)`

Predicts the center position associated with a given chord.

**Parameters**:
- `chord` (str): The chord symbol for which to predict the center position.

**Returns**:
- `center` (tuple): A tuple representing the predicted center position (x, y) associated with the given chord.

**Description**:
This function predicts the center position associated with a given chord based on predefined mappings between chords and center positions. The function uses the chord symbol to look up the corresponding center position and returns it as a tuple.


#### `DensityFunction`

Represents a density function used for calculating density values based on different types of distributions.

**Parameters**:
- `color` (str): The hex CMY color value associated with the density function.
- `center` (tuple, optional): The center of the density function. Default is `None`.
- `variance` (tuple, optional): The variance of the density function. Default is `None`.
- `func` (callable, optional): A custom function to calculate the density. Default is `None`.
- `type` (str, optional): The type of the density function. Possible values are `'uniform'` or `'gaussian'`. Default is `'gaussian'`.


**Attributes**:
- `type` (str): The type of the density function.
- `color` (str): The hex CMY color value associated with the density function.
- `center` (tuple): The center of the density function.
- `variance` (tuple): The variance of the density function.
- `func` (callable): The function to calculate the density.

**Description**:
This class represents a density function used for calculating density values based on different types of distributions. It supports both uniform and Gaussian distributions. If a custom function is provided, it will be used to calculate the density values.
