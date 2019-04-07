// Fluid.cpp : This file contains the 'main' function. Program execution begins and ends there.
//

#include <iostream>
#include <glm/glm.hpp>

#include "Display.h"
#include "Fluid.h"

// Clamps val between min and max
#define CLAMP(val, min, max) (val < min ? min : (val > max ? max : val))

const int N = 256 + 128;

using namespace std;
using namespace glm;

enum PlacingMode { DENSITY, OBSTACLE };

int main(int argc, char** argv)
{
	Display display(900, 900, N, N, "Fluid Sim");

	Fluid fluid(N, 1.0f / 60.0f, 0.0000000f, 1e-10f, 0.0001f, 23.0f);

	Uint64 now = SDL_GetPerformanceCounter();
	Uint64 last = 0;
	float deltaTime = 0;

	float scale_x = (float)N / display.GetWindowWidth();
	float scale_y = (float)N / display.GetWindowHeight();

	int mouse_x, mouse_y;
	int prev_mouse_x = 0, prev_mouse_y = 0;

	bool mouse_down = false;


	PlacingMode placing_mode = PlacingMode::DENSITY;

	RenderMode render_mode = RenderMode::RENDER_TEMPERATURE;
	
	SDL_Event event;
	bool close_window = false;

	while (!close_window)
	{
		// Handle SDL events
		while (SDL_PollEvent(&event))
		{
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
						case SDLK_r: 
						{
							render_mode = (RenderMode)((render_mode + 1) % 2);
							
							display.SetTitle("Fluid Sim - Rendermode: " + Render_Mode_Names[render_mode]);

							break;
						}
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
					int x_prev = CLAMP((int)(prev_mouse_x * scale_x), 1, N - 2);
					int y_prev = CLAMP((int)(prev_mouse_y * scale_y), 1, N - 2);

					if (x == x_prev)
					{
						int y0 = min(y, y_prev);
						int y1 = max(y, y_prev);

						for (int j = y0; j <= y1; j++)
						{
							fluid.AddObstacle(x, j);
						}
					}
					else
					{
						// Bresenham's line algorithm. Based on: https://en.wikipedia.org/wiki/Bresenham%27s_line_algorithm
						float deltax = x - x_prev;
						float deltay = y - y_prev;
						float deltaerr = abs(deltay / deltax);

						float error = 0.0f;
						int j = y_prev;

						int x0 = min(x, x_prev);
						int x1 = max(x, x_prev);

						for (int i = x0; i <= x1; i++)
						{
							fluid.AddObstacle(i, j);

							error += deltaerr;

							if (error >= 0.5f)
							{
								y += sign(deltay);
								error -= 1.0f;
							}
						}
					}
					
					break;
				}
			}
		}
		
		fluid.Update();
		fluid.Render(display, render_mode);

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
