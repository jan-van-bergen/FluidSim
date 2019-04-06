// Fluid.cpp : This file contains the 'main' function. Program execution begins and ends there.
//

#include <iostream>
#include <glm/glm.hpp>

#include "Display.h"
#include "Fluid.h"

// Clamps val between min and max
#define CLAMP(val, min, max) (val < min ? min : (val > max ? max : val))

const int N = 256;

using namespace std;
using namespace glm;

enum PlacingMode { DENSITY, OBSTACLE };

int main(int argc, char** argv)
{
	Display display(512, 512, N, N, "Fluid Simulation Test");

	Fluid fluid(N, 1.0f / 60.0f, 0.01f, 0.01f);

	Uint64 now = SDL_GetPerformanceCounter();
	Uint64 last = 0;
	float deltaTime = 0;

	float scale_x = (float)N / display.GetWindowWidth();
	float scale_y = (float)N / display.GetWindowHeight();

	int mouse_x, mouse_y;
	int prev_mouse_x = 0, prev_mouse_y = 0;

	bool mouse_down = false;

	PlacingMode mode = PlacingMode::DENSITY;

	SDL_Event event;

	while (!display.IsClosed())
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
						case SDLK_d: mode = PlacingMode::DENSITY;  break;
						case SDLK_o: mode = PlacingMode::OBSTACLE; break;
					}

					break;
				}
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
			switch (mode)
			{
				case PlacingMode::DENSITY:
				{
					for (int j = -size; j <= size; j++)
					{
						for (int i = -size; i <= size; i++)
						{
							fluid.AddDensity(x + i, y + j, 1);
							fluid.AddVelocity(x + i, y + j, delta_x, delta_y);
						}
					}

					break;
				}

				case PlacingMode::OBSTACLE:
				{
					fluid.AddObstacle(mouse_x, mouse_y);

					break;
				}
			}
		}
		
		fluid.Update();
		fluid.Render(display);

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

// Run program: Ctrl + F5 or Debug > Start Without Debugging menu
// Debug program: F5 or Debug > Start Debugging menu

// Tips for Getting Started: 
//   1. Use the Solution Explorer window to add/manage files
//   2. Use the Team Explorer window to connect to source control
//   3. Use the Output window to see build output and other messages
//   4. Use the Error List window to view errors
//   5. Go to Project > Add New Item to create new code files, or Project > Add Existing Item to add existing code files to the project
//   6. In the future, to open this project again, go to File > Open > Project and select the .sln file
