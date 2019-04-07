#include <math.h>
#include <stdlib.h>
#include "Fluid.h"

// Number of iterations used by Gauss-Seidel
#define ITERATIONS 4

// Get the 1d index from two 2d indices
#define INDEX(x, y) ((x) + (y) * size)

// Clamps val between min and max
#define CLAMP(val, min, max) (val < min ? min : (val > max ? max : val))

using namespace std;
using namespace glm;

Fluid::Fluid(int size, float delta_time, float diffusion, float viscosity, float room_temperature) : 
	size(size), 
	delta_time(delta_time), 
	diffusion(diffusion), 
	viscosity(viscosity), 
	room_temperature(room_temperature)
{
	// Allocate grids
	const int mem_size = size * size;

	density      = new float[mem_size];
	density_prev = new float[mem_size];

	temperature      = new float[mem_size];
	temperature_prev = new float[mem_size];

	velocity_x      = new float[mem_size];
	velocity_x_prev = new float[mem_size];

	velocity_y      = new float[mem_size];
	velocity_y_prev = new float[mem_size];

	obstacle = new bool[mem_size];

	// Initialize all the fields
	for (int i = 0; i < mem_size; i++)
	{
		density[i]      = 0;
		density_prev[i] = 0;

		temperature[i]      = room_temperature;
		temperature_prev[i] = room_temperature;

		velocity_x[i]      = 0;
		velocity_x_prev[i] = 0;

		velocity_y[i]      = 0;
		velocity_y_prev[i] = 0;

		obstacle[i] = false;
	}

	// Load colour palette from file
	SDL_Surface* palette_bmp = SDL_LoadBMP("../MiniProject/Data/Colour_palette.bmp");

	unsigned char* pixels = (unsigned char*)palette_bmp->pixels;

	palette_size = palette_bmp->w;
	palette      = new vec3[palette_size];

	for (int i = 0; i < palette_size; i++)
	{
		// The file is stored in BGR format
		unsigned char b = pixels[i * 3];
		unsigned char g = pixels[i * 3 + 1];
		unsigned char r = pixels[i * 3 + 2];

		palette[i] = vec3(r / 255.0f, g / 255.0f, b / 255.0f);
	}

	SDL_FreeSurface(palette_bmp);
}

Fluid::~Fluid()
{
	delete[] density;
	delete[] density_prev;

	delete[] temperature;
	delete[] temperature_prev;

	delete[] velocity_x;
	delete[] velocity_x_prev;

	delete[] velocity_y;
	delete[] velocity_y_prev;

	delete[] palette;
}

void Fluid::AddDensity(int x, int y, float amount)
{
	const int index = INDEX(x, y);

	if (obstacle[index]) return;

	density[index] += amount;
}

void Fluid::AddVelocity(int x, int y, float amountX, float amountY)
{
	const int index = INDEX(x, y);

	if (obstacle[index]) return;

	velocity_x[index] += amountX;
	velocity_y[index] += amountY;
}

void Fluid::AddTemperature(int x, int y, float amount)
{
	const int index = INDEX(x, y);

	if (obstacle[index]) return;

	temperature[index] += amount;
}

void Fluid::AddObstacle(int x, int y)
{
	// Ensure that obstacles are never isolated as this is an assumption made by the boundary solver
	obstacle[INDEX(x,     y    )] = true;
	obstacle[INDEX(x + 1, y    )] = true;
	obstacle[INDEX(x,     y + 1)] = true;
	obstacle[INDEX(x + 1, y + 1)] = true;
}

void Fluid::RemoveObstacle(int x, int y)
{
	// Ensure that obstacles are never isolated as this is an assumption made by the boundary solver
	obstacle[INDEX(x,     y    )] = false;
	obstacle[INDEX(x + 1, y    )] = false;
	obstacle[INDEX(x,     y + 1)] = false;
	obstacle[INDEX(x + 1, y + 1)] = false;
}

// Update the fluid simulation a single time step
void Fluid::Update()
{
	// Diffuse the velocity field according to viscosity
	Diffuse(VELOCITY_X, velocity_x_prev, velocity_x, viscosity);
	Diffuse(VELOCITY_Y, velocity_y_prev, velocity_y, viscosity);

	// Force the velocities to be mass conserving
	Project(velocity_x_prev, velocity_y_prev, velocity_x, velocity_y);

	// Advect behaves more accurately when the velocity field is mass conserving
	Advect(VELOCITY_X, velocity_x, velocity_x_prev, velocity_x_prev, velocity_y_prev);
	Advect(VELOCITY_Y, velocity_y, velocity_y_prev, velocity_x_prev, velocity_y_prev);

	// Apply bouyancy force to velocity
	Buoyancy();

	Project(velocity_x, velocity_y, velocity_x_prev, velocity_y_prev);

	Diffuse(DIFFUSE, density_prev, density, diffusion);
	Advect(DIFFUSE, density, density_prev, velocity_x, velocity_y);

	Diffuse(DIFFUSE, temperature_prev, temperature, 0.00001f);
	Advect(DIFFUSE, temperature, temperature_prev, velocity_x, velocity_y);
}

// Render the fluid simulation to the display
void Fluid::Render(Display& display)
{
	const float scale_x = display.GetBufferWidth()  / (float)size;
	const float scale_y = display.GetBufferHeight() / (float)size;

	vec3* screen = display.GetBuffer();
	const int width = display.GetBufferWidth();

	for (int j = 1; j < size - 1; j++)
	{
		for (int i = 1; i < size - 1; i++)
		{
			if (obstacle[INDEX(i, j)])
			{
				screen[(int)(i * scale_x + j * scale_y * width)] = vec3(0.6f, 0.2f, 0.2f);
			}
			else
			{
				const int temp = CLAMP((int)temperature[INDEX(i, j)], 0, palette_size - 1);

				screen[(int)(i * scale_x + j * scale_y * width)] = palette[temp];
			}
		}
	}
}

void Fluid::Diffuse(const Boundary b, float* x, float* x0, const float amount)
{
	const float a = delta_time * amount * (size - 2) * (size - 2);

	GaussSeidel(b, x, x0, a, 1 + 4 * a);
}

// Every velocity field is the sum of an incompressible field and a gradient field
// To obtain an incompressible field we subtract the gradient field from our current velocities
// Stores pressure in p and divergence in div
void Fluid::Project(float* vel_x, float* vel_y, float* p, float* div)
{
	const float factor = -0.5f / size;

	for (int j = 1; j < size - 1; j++) 
	{
		for (int i = 1; i < size - 1; i++) 
		{
			// Take divergence
			div[INDEX(i, j)] = factor * (vel_x[INDEX(i + 1, j    )] - vel_x[INDEX(i - 1, j    )] +
									     vel_y[INDEX(i,     j + 1)] - vel_y[INDEX(i,     j - 1)]);
			// Initialize pressure to zero
			p[INDEX(i, j)] = 0;
		}
	}

	SetBound(DIFFUSE, div);
	SetBound(DIFFUSE, p);

	// Solve for pressure
	GaussSeidel(DIFFUSE, p, div, 1, 4);

	for (int j = 1; j < size - 1; j++) 
	{
		for (int i = 1; i < size - 1; i++) 
		{
			vel_x[INDEX(i, j)] -= 0.5f * (p[INDEX(i + 1, j    )] - p[INDEX(i - 1, j    )]) * size;
			vel_y[INDEX(i, j)] -= 0.5f * (p[INDEX(i,     j + 1)] - p[INDEX(i,     j - 1)]) * size;
		}
	}

	SetBound(VELOCITY_X, vel_x);
	SetBound(VELOCITY_Y, vel_y);
}

// Semi-Lagrangian advection
// Advect field d from field d0 according to the x and y velocities in the grid
void Fluid::Advect(const Boundary b, float* d, float* d0, const float* vel_x, const float* vel_y)
{
	const float dt_horizontal = delta_time * (size - 2);
	const float dt_vertical   = delta_time * (size - 2);
	
	const float edge = size + 0.5f;

	for (int j = 1; j < size - 1; j++) 
	{
		for (int i = 1; i < size - 1; i++) 
		{
			// Move backward according to current velocities
			float x = i - dt_horizontal * vel_x[INDEX(i, j)];
			float y = j - dt_vertical   * vel_y[INDEX(i, j)];
			
			// Clamp x and y to be valid
			x = CLAMP(x, 0.5f, edge);
			y = CLAMP(y, 0.5f, edge);

			// Find nearest corners
			float i0 = floorf(x);
			float j0 = floorf(y);

			// Check if the indices are valid
			// We want to ensure that i0, j0, i1, j1 < 256 
			// This means we only need to check when i0, j0 >= 255 because i1 = i0 + 1 and j1 = j0 + 1
			if (i0 >= 255.0f || j0 >= 255.0f) continue;

			float i1 = i0 + 1.0f;
			float j1 = j0 + 1.0f;

			// Find interpolation factors
			float s1 = x - i0;
			float s0 = 1 - s1;
			float t1 = y - j0;
			float t0 = 1 - t1;
			
			// Cast to int for indexing
			int i0i = (int)i0;
			int i1i = (int)i1;
			int j0i = (int)j0;
			int j1i = (int)j1;

			d[INDEX(i, j)] =
				s0 * (t0 * d0[INDEX(i0i, j0i)] + t1 * d0[INDEX(i0i, j1i)]) +
				s1 * (t0 * d0[INDEX(i1i, j0i)] + t1 * d0[INDEX(i1i, j1i)]);
		}
	}

	SetBound(b, d);
}

void Fluid::SetBound(const Boundary b, float* x)
{
	// Handle top and bottom boundaries
	for (int i = 1; i < size - 1; i++)
	{
		// Copy if x is not velocity_y or velocity_y prev, otherwise negate
		x[INDEX(i, 0       )] = b == Boundary::VELOCITY_Y ? -x[INDEX(i, 1       )] : x[INDEX(i, 1       )];
		x[INDEX(i, size - 1)] = b == Boundary::VELOCITY_Y ? -x[INDEX(i, size - 2)] : x[INDEX(i, size - 2)];
	}

	// Handle left and right boundaries
	for (int j = 1; j < size - 1; j++)
	{
		// Copy if x is not velocity_x or velocity_x prev, otherwise negate
		x[INDEX(0,        j)] = b == Boundary::VELOCITY_X ? -x[INDEX(1,        j)] : x[INDEX(1,        j)];
		x[INDEX(size - 1, j)] = b == Boundary::VELOCITY_X ? -x[INDEX(size - 2, j)] : x[INDEX(size - 2, j)];
	}

	// Handle the corners of the boundary
	x[INDEX(0,        0       )] = 0.5f * (x[INDEX(1,        0       )] + x[INDEX(0,        1       )]);
	x[INDEX(0,        size - 1)] = 0.5f * (x[INDEX(1,        size - 1)] + x[INDEX(0,        size - 2)]);
	x[INDEX(size - 1, 0       )] = 0.5f * (x[INDEX(size - 2, 0       )] + x[INDEX(size - 1, 1       )]);
	x[INDEX(size - 1, size - 1)] = 0.5f * (x[INDEX(size - 2, size - 1)] + x[INDEX(size - 1, size - 2)]);

	// Handle obstacles in the grid
	for (int j = 1; j < size - 1; j++)
	{
		for (int i = 1; i < size - 1; i++)
		{
			const int ij = INDEX(i, j);

			if (obstacle[ij])
			{
				switch (b)
				{
					case Boundary::VELOCITY_X:
					{
						if (!obstacle[INDEX(i - 1, j)])
						{
							x[ij] = -x[INDEX(i - 1, j)];
						}
						else if (!obstacle[INDEX(i + 1, j)])
						{
							x[ij] = -x[INDEX(i + 1, j)];
						}
						else
						{
							// @TODO: optimize indices
							x[ij] = 0.5f * (x[INDEX(i - 1, j)] + x[INDEX(i + 1, j)]);
						}

						break;
					}

					case Boundary::VELOCITY_Y:
					{
						if (!obstacle[INDEX(i, j - 1)])
						{
							x[ij] = -x[INDEX(i, j - 1)];
						}
						else if (!obstacle[INDEX(i, j + 1)])
						{
							x[ij] = -x[INDEX(i, j + 1)];
						}
						else
						{
							// @TODO: optimize indices
							x[ij] = 0.5f * (x[INDEX(i, j - 1)] + x[INDEX(i, j + 1)]);
						}

						break;
					}

					case Boundary::DIFFUSE:
					{
						//x[ij] = 0;

						break;
					}
				}
			}
		}
	}
}

void Fluid::Buoyancy()
{
	const float P = 1000;
	const float m = 0.001f;
	const float g = 9.81f;

	const float R = 8.314472f;

	const float c = P * m * g / R;
	const float inv_room_temp = 1.0f / room_temperature;
	
	for (int j = 1; j < size - 1; j++)
	{
		for (int i = 1; i < size - 1; i++)
		{
			const int ij = INDEX(i, j);

			// F_buoyancy = P*m*g/R * (1 / T_0 - 1 / T) * u, where u is the up vector
			const float buoyancy = c * (inv_room_temp - 1.0f / temperature[ij]);

			// Apply buoyancy force in the up direction
			velocity_y[ij] -= buoyancy * delta_time;
		}
	}
}

// Gauss-Seidel method to iteratively solve linear systems
// x is what we are trying to solve for
void Fluid::GaussSeidel(const Boundary b, float* x, float* x0, const float a, const float c)
{
	const float inv_c = 1.0f / c;

	for (int k = 0; k < ITERATIONS; k++)
	{
		for (int j = 1; j < size - 1; j++) 
		{
			for (int i = 1; i < size - 1; i++) 
			{
				x[INDEX(i, j)] = inv_c * (x0[INDEX(i, j)]+ a * (x[INDEX(i + 1, j    )] +
												                x[INDEX(i - 1, j    )] +
												                x[INDEX(i,     j + 1)] +
												                x[INDEX(i,     j - 1)]));
			}
		}

		SetBound(b, x);
	}
}