# Fluid Simulation Sandbox

2D semi-lagrangian fluid solver based on "Stable Fluids" solver by Jos Stam.

Users can interact in real-time with the fluid, using various controls.
We also provide various render modes to provide users a better understanding of what is going on within the simulation.

## Features
- Real time fluid interaction.
- External forces such as gravity and buoyancy.
- Internal boundaries that you can draw.
- Vorticity confinement
- OpenMP parallel code

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

## Building the Project

Headers and lib's are included in the project and should work out of the box.
2 DLL's are required for the executable to work. These are glew32.dll and SDL.dll. They are currently in the root folder of the git repository, but should be placed in the same folder as the .exe (This is the ./Release folder when building with Visual Studio).
<b>NOTE: Our SDL version does not seem work with the x64 platform WHICH IS THE DEFAULT SOLUTION PLATFORM WHEN OPENING THE SOLUTION WITH VISUAL STUDIO! It should be changed to x86.</b>
The project uses a Visual Studio 2017 solution.

(This git repository was originally hosted on Gitlab.)
