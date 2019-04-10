// Fluid.cpp : This file contains the 'main' function. Program execution begins and ends there.
//

#include <iostream>
#include <glm/glm.hpp>
#include <Imgui/imgui.h>
#include <Imgui/imgui_impl_sdl.h>

#include "Display.h"
#include "Fluid.h"

// Clamps val between min and max
#define CLAMP(val, min, max) (val < min ? min : (val > max ? max : val))

const int N = 256 + 128;

using namespace std;
using namespace glm;

enum PlacingMode { FIRE_BALL, DENSITY, TEMPERATURE, VELOCITY, OBSTACLE };
const string PlacingMode_Names[5] = { "Fire Ball", "Density", "Temperature", "Velocity", "Obstacle" };

inline void NextRenderMode(RenderMode& render_mode)
{
	render_mode = (RenderMode)((render_mode + 1) % 3);
}

int main(int argc, char** argv)
{
	Display display(900, 900, N, N, "Fluid Sim");

	Fluid fluid(N, 1.0f / 30.0f, 0.0f, 1e-10f, 0.0001f, 23.0f, 5.0f);

	Uint64 now = SDL_GetPerformanceCounter();
	Uint64 last = 0;
	float deltaTime = 0;

	const float scale_x = (float)N / display.GetWindowWidth();
	const float scale_y = (float)N / display.GetWindowHeight();

	int mouse_x, mouse_y;
	int prev_mouse_x = 0, prev_mouse_y = 0;

	bool mouse_down = false;

	bool wind = false;
	
	PlacingMode placing_mode = PlacingMode::FIRE_BALL;

	RenderMode render_mode = RenderMode::RENDER_DENSITY;
	NextRenderMode(render_mode);
	
	SDL_Event event;
	bool close_window = false;

	while (!close_window)
	{
		// Handle SDL events
		while (SDL_PollEvent(&event))
		{
			ImGui_ImplSDL2_ProcessEvent(&event);

			switch (event.type)
			{
				// Mouse up / down
				case SDL_MOUSEBUTTONDOWN: if (!ImGui::GetIO().WantCaptureMouse) mouse_down = true;  break;
				case SDL_MOUSEBUTTONUP:   if (!ImGui::GetIO().WantCaptureMouse) mouse_down = false; break;

				case SDL_KEYUP:
				{
					switch (event.key.keysym.sym)
					{
						// Change placing mode
						case SDLK_f: placing_mode = PlacingMode::FIRE_BALL;   break;
						case SDLK_d: placing_mode = PlacingMode::DENSITY;     break;
						case SDLK_t: placing_mode = PlacingMode::TEMPERATURE; break;
						case SDLK_v: placing_mode = PlacingMode::VELOCITY;    break;
						case SDLK_o: placing_mode = PlacingMode::OBSTACLE;    break;

						case SDLK_w:
						{
							wind = !wind;

							if (wind)
							{
								// Turn off gravity and buoyancy in wind tunnel mode
								fluid.kappa = 0;
								fluid.sigma = 0;
							}

							break;
						}

						// Change render mode
						case SDLK_SPACE: NextRenderMode(render_mode); break;

						// Reset grid
						case SDLK_z: fluid.Reset(); break;
					}

					break;
				}

				case SDL_QUIT: close_window = true; break;
			}
		}

		display.Clear();

		SDL_GetMouseState(&mouse_x, &mouse_y);

		int x = CLAMP((int)(mouse_x * scale_x), 1, N - 2);
		int y = CLAMP((int)(mouse_y * scale_y), 1, N - 2);

		float delta_x = CLAMP(mouse_x - prev_mouse_x, -10, 10);
		float delta_y = CLAMP(mouse_y - prev_mouse_y, -10, 10);

		const int size = 1;

		if (mouse_down)
		{
			switch (placing_mode)
			{
				// Add some density, temperature and velocity in the region around the user's mouse
				case PlacingMode::FIRE_BALL:
				{
					for (int j = -size; j <= size; j++)
					{
						for (int i = -size; i <= size; i++)
						{
							fluid.AddDensity(x + i, y + j, 0.5f);
							fluid.AddTemperature(x + i, y + j, 1000.0f);
							fluid.AddVelocity(x + i, y + j, delta_x, delta_y - 0.5f);
						}
					}

					break;
				}

				case PlacingMode::DENSITY:
				{
					for (int j = -size; j <= size; j++)
					{
						for (int i = -size; i <= size; i++)
						{
							fluid.AddDensity(x + i, y + j, 0.5f);
						}
					}

					break;
				}

				case PlacingMode::TEMPERATURE:
				{
					for (int j = -size; j <= size; j++)
					{
						for (int i = -size; i <= size; i++)
						{
							fluid.AddTemperature(x + i, y + j, 1000.0f);
						}
					}

					break;
				}

				case PlacingMode::VELOCITY:
				{
					for (int j = -size; j <= size; j++)
					{
						for (int i = -size; i <= size; i++)
						{
							fluid.AddVelocity(x + i, y + j, delta_x, delta_y);
						}
					}

					break;
				}

				// Place obstacles along the line from the previous mouse coordinates to the current
				case PlacingMode::OBSTACLE:
				{
					fluid.AddObstacle(x, y);
					
					break;
				}
			}
		}

		if (wind)
		{
			for (int i = 10; i < N - 1; i += 20)
			{
				fluid.AddDensity(10, i, 1);
				fluid.AddTemperature(10, i, 1000);
				fluid.AddVelocity(10, i, 10, 0);
			}
		}

		fluid.Update();
		fluid.Render(display, render_mode);

		display.StartGUI();

		ImGui::Begin("Fluid Simulation");                          

		if (ImGui::CollapsingHeader("Parameters", ImGuiTreeNodeFlags_DefaultOpen))
		{
			ImGui::SliderFloat("Viscosity",             &fluid.viscosity,             0.0f, 0.1f);
			ImGui::SliderFloat("Density Diffusion",     &fluid.density_diffusion,     0.0f, 0.1f);
			ImGui::SliderFloat("Temperature Diffusion", &fluid.temperature_diffusion, 0.0f, 0.1f);

			ImGui::SliderFloat("Room Temperature", &fluid.room_temperature, 0.1f, 100.0f);

			ImGui::SliderFloat("Gravity",  &fluid.kappa,  0.0f, 1.0f);
			ImGui::SliderFloat("Buoyancy", &fluid.sigma, -1.0f, 0.0f);

			ImGui::SliderFloat("Vorticity Confinement", &fluid.vorticity_confinement, 0.0f, 20.0f);
		}

		if (ImGui::CollapsingHeader("Info", ImGuiTreeNodeFlags_DefaultOpen))
		{
			const string& w = wind ? "On" : "Off";

			ImGui::Text("State:");
			ImGui::Text("\tFPS: %.1f", ImGui::GetIO().Framerate);
			ImGui::Text(("\tRendermode: " + RenderMode_Names[render_mode]).c_str());
			ImGui::Text(("\tPlacing: " + PlacingMode_Names[placing_mode]).c_str());
			ImGui::Text(("\tWind Tunnel: " + w).c_str());

			ImGui::Text("Controls:");
			ImGui::Text("\tF - Fire Ball placing mode");
			ImGui::Text("\tD - Density placing mode");
			ImGui::Text("\tT - Temperature placing mode");
			ImGui::Text("\tV - Velocity placing mode");
			ImGui::Text("\tO - Obstacle placing mode");
			ImGui::Text("\tW - Toggle wind tunnel mode");	
			ImGui::Text("\tR - Reset grid");
			ImGui::Text("\tSPACE - Change render mode");
		}

		ImGui::End();

		display.Update();

		last = now;
		now = SDL_GetPerformanceCounter();

		deltaTime = ((now - last) * 1000 / (float)SDL_GetPerformanceFrequency());
		printf("Delta time: %f ms\n", deltaTime);

		prev_mouse_x = mouse_x;
		prev_mouse_y = mouse_y;
	}

	return 0;
}
