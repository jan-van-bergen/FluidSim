# Fluid Simulation Sandbox

2D semi-lagrangian fluid solver based on "Stable Fluids" solver by Jos Stam.

Users can interact in real-time with the fluid, using various controls.
We also provide various render modes to provide users a better understanding of what is going on within the simulation.

## Features
- Real time fluid interaction.
- External forces such as gravity and buoyancy.
- Internal boundaries that you can draw.
- Vorticity confinement

## Screenshots

#### Density Render Mode
![Density](ScreenShots/Density.png "Density")

#### Windtunnel Obstacle
![Windtunnel Obstacle](ScreenShots/Fire_in_wind_shielded_by_obstacle.png "Windtunnel Obstacle")

#### Windtunnel
![Windtunnel](ScreenShots/Fire_in_wind.png "Windtunnel")

#### Temperature into Obstacle
![Obstacle](ScreenShots/Fire_into_obstacle.png "Obstacle")

#### Temperature Render Mode
![Fire](ScreenShots/Fire.png "Fire")

#### Velocity Render Mode
![Motion](ScreenShots/Motion.png "Motion")

## Dependencies
- SDL 2
- GLEW
- Dear Imgui
- glm

DLL's, headers and lib's are included in the project and should work out of the box.
The project uses a Visual Studio 2017 solution.

This git repository was originally hosted on Gitlab.
