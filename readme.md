# Fluid Simulation Sandbox

2D semi-lagrangian fluid solver based on "Stable Fluids" solver by Jos Stam.

## Features
- Real time fluid interaction.
- External forces such as gravity and buoyancy.
- Internal boundaries that you can draw.
- Vorticity confinement

## Screenshots

#### Density Render Mode
![Density](Screenshots/Density.png "Density")

#### Windtunnel Obstacle
![Windtunnel Obstacle](Screenshots/Fire in wind shielded by obstacle.png "Windtunnel Obstacle")

#### Windtunnel
![Windtunnel](Screenshots/Fire in wind.png "Windtunnel")

#### Temperature into Obstacle
![Obstacle](screenshots/Fire into obstacle.png "Obstacle")

#### Temperature Render Mode
![Fire](screenshots/Fire.png "Fire")

#### Velocity Render Mode
![Motion](screenshots/Motion.png "Motion")

## Dependencies
- SDL 2
- GLEW
- Dear Imgui
- glm

DLL's, headers and lib's are included in the project and should work out of the box.
The project uses a Visual Studio 2017 solution.
