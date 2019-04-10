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

enum PlacingMode { DENSITY, OBSTACLE };
const string PlacingMode_Names[2] = { "Density", "Obstacle" };

void NextRenderMode(Display& display, RenderMode& render_mode)
{
	render_mode = (RenderMode)((render_mode + 1) % 3);

	display.SetTitle("Fluid Sim - Rendermode: " + RenderMode_Names[render_mode]);
}

int main(int argc, char** argv)
{
	// @TEMP
	bool show_demo_window = true;
	bool show_another_window = false;
	float f;
	int counter = 0;
	ImVec4 clear_color = ImVec4(0.45f, 0.55f, 0.60f, 1.00f);
	// @TEMP

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

	PlacingMode placing_mode = PlacingMode::DENSITY;

	RenderMode render_mode = RenderMode::RENDER_DENSITY;
	NextRenderMode(display, render_mode);
	
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
				case SDL_MOUSEBUTTONDOWN: mouse_down = true;  break;
				case SDL_MOUSEBUTTONUP:   mouse_down = false; break;

				case SDL_KEYUP:
				{
					switch (event.key.keysym.sym)
					{
						// Change placing mode
						case SDLK_d: placing_mode = PlacingMode::DENSITY;  break;
						case SDLK_o: placing_mode = PlacingMode::OBSTACLE; break;

						// Change render mode
						case SDLK_SPACE: NextRenderMode(display, render_mode); break;

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
				// Add some density and velocity in the region around the user's mouse
				case PlacingMode::DENSITY:
				{
					for (int j = -size; j <= size; j++)
					{
						for (int i = -size; i <= size; i++)
						{
							fluid.AddDensity(x + i, y + j, 0.5f);
							fluid.AddTemperature(x + i, y + j, 1000.0f);
							fluid.AddVelocity(x + i, y + j, delta_x, delta_y - 1.5f);
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

			ImGui::SliderFloat("Gravity",  &fluid.kappa,  0.0f, 10.0f);
			ImGui::SliderFloat("Buoyancy", &fluid.sigma, -10.0f, 0.0f);

			ImGui::SliderFloat("Vorticity Confinement", &fluid.vorticity_confinement, 0.0f, 20.0f);
		}

		if (ImGui::CollapsingHeader("Info", ImGuiTreeNodeFlags_DefaultOpen))
		{
			ImGui::Text(("Rendermode: " + RenderMode_Names[render_mode]).c_str());
			ImGui::Text("Change render mode with SPACEBAR");

			ImGui::Text(("Placing: " + PlacingMode_Names[placing_mode]).c_str());
			ImGui::Text("Change placing mode with D and O");
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
