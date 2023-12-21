<div align="center">

# Computational Simulation of Energy Loss in a Formula One Circuit

##### MATLAB simulation of an F1 car navigating through a curved track. Applies numeric methods and principles of energy conservation.

![MATLAB](https://www.mathworks.com/matlabcentral/images/matlab-file-exchange.svg)

![Example Image](logo.png)

</div>

## Overview

This project involves the development of a MATLAB simulation showcasing the practical application of fundamental principles of energy conservation for system analysis. Initially, a track curve for a Formula 1 circuit was designed using Lagrange polynomial interpolation. The resulting curve is a third-degree polynomial. Additionally, in designing this track segment, consideration was given to an appropriate banking angle to ensure the safety of the drivers. Subsequently, principles related to energy conservation, friction, momentum, and rigid body rotation were employed to create a Graphical User Interface (GUI). The GUI enables users to observe energy changes, trajectory variations, and velocity fluctuations of a F1 car navigating the designed curve zone. For the calculation of dynamic variables such as the car's velocity, the Euler method was utilized. The simulation incorporates a skid condition, allowing visualization of the effects when a car enters the curve zone at a speed exceeding the recommended limit. Furthermore, the simulation facilitates the investigation of energy dissipation through elastic, inelastic, and perfectly inelastic collisions.

## Repository Contents

### Image Files
The repository includes a set of essential `.jpg` and `.png` files, integral to the graphical user interface (GUI) design. It is crucial not to alter their names or locations, as they are specifically utilized for the visual representation of the F1 car racing simulation.

### Main Scripts

1. **SimpleTest.m**
   - *Description:* This file is a standard MATLAB script designed to draw the curves of the track that the F1 car will navigate. It provides a straightforward visualization of the racing path without considering complex simulations.

2. **TestWithoutAirResistance.m**
   - *Description:* This script builds upon the calculations introduced in `SimpleTest.m` and extends the simulation to incorporate various factors. However, it specifically excludes considerations for air resistance and collision scenarios. It serves as an intermediate step in understanding the car's behavior under basic conditions.

3. **FinalRawCode.m**
   - *Description:* The `FinalRawCode.m` script represents the culminated version of the simulation, excluding any graphical user interface elements. It is recommended for users who wish to focus solely on testing the underlying calculations and reviewing simulation results without additional visual elements.

4. **FinalGUIF1.mlapp**
   - *Description:* For users preferring a simplified and aesthetically pleasing experience, `FinalGUIF1.mlapp` is provided. This MATLAB application utilizes the same underlying code as `FinalRawCode.m` but introduces a user-friendly GUI for enhanced interaction. It is the recommended script for those who value a more interactive and visually appealing exploration of the F1 car simulation.

Feel free to explore and utilize the scripts based on your specific needs, ranging from basic curve visualization to comprehensive simulations with user interface integration.


## Usage

Follow these steps to get the project up and running on your local machine.

### Prerequisites

Make sure you have the following installed on your machine:

- [Git](https://git-scm.com/)
- [MATLAB](https://matlab.mathworks.com/)

### Installation

- Clone the repository:

    ```bash
    git clone https://github.com/HumbertoBM2/Computational-Simulation-of-Energy-Loss-in-a-Formula-One-Circuit
    ```

- Open the cloned repository using MATLAB. Run the simulation and enjoy. 