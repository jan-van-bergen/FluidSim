#include <math.h>
#include <stdlib.h>
#include "Fluid.h"

// Number of iterations used by Gauss-Seidel
#define ITERATIONS 4
// Wall Thickness. Should always be >= 2 as this is an assumption made by the boundary solver
#define WALL_THICKNESS 4

// Get the 1d index from two 2d indices
#define INDEX(x, y) ((x) + (y) * size)

// Clamps val between min and max
#define CLAMP(val, min, max) (val < min ? min : (val > max ? max : val))

using namespace std;
using namespace glm;

Fluid::Fluid(int size, float delta_time, float viscosity, float density_diffusion, float temperature_diffusion, float room_temperature) :
	size(size), 
	viscosity(viscosity), 
	delta_time(delta_time), 
	density_diffusion(density_diffusion),
	temperature_diffusion(temperature_diffusion),
	room_temperature(room_temperature)
{
	// Allocate grids
	const int mem_size = size * size;

	density      = new float[mem_size];
	density_copy = new float[mem_size];

	temperature      = new float[mem_size];
	temperature_copy = new float[mem_size];

	velocity_x      = new float[mem_size];
	velocity_x_copy = new float[mem_size];

	velocity_y      = new float[mem_size];
	velocity_y_copy = new float[mem_size];

	obstacle = new bool[mem_size];

	// Initialize all the fields
	Reset();

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
	delete[] density_copy;

	delete[] temperature;
	delete[] temperature_copy;

	delete[] velocity_x;
	delete[] velocity_x_copy;

	delete[] velocity_y;
	delete[] velocity_y_copy;

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
	for (int j = y; j < y + WALL_THICKNESS; j++)
	{
		for (int i = x; i < x + WALL_THICKNESS; i++)
		{
			obstacle[INDEX(i, j)] = true;
		}
	}
}

void Fluid::RemoveObstacle(int x, int y)
{
	for (int j = y; j < y + WALL_THICKNESS; j++)
	{
		for (int i = x; i < x + WALL_THICKNESS; i++)
		{
			obstacle[INDEX(i, j)] = false;
		}
	}
}

// Update the fluid simulation a single time step
void Fluid::Update()
{
	// Diffuse the velocity field according to viscosity
	Diffuse(VELOCITY_X, velocity_x_copy, velocity_x, viscosity);
	Diffuse(VELOCITY_Y, velocity_y_copy, velocity_y, viscosity);

	// Project to incompressible fluid
	Project(velocity_x_copy, velocity_y_copy, velocity_x, velocity_y);

	// Advect behaves more accurately when the velocity field is mass conserving
	Advect(VELOCITY_X, velocity_x, velocity_x_copy, velocity_x_copy, velocity_y_copy);
	Advect(VELOCITY_Y, velocity_y, velocity_y_copy, velocity_x_copy, velocity_y_copy);

	// Apply external forces
	ExternalForces();

	// Project to incompressible fluid
	Project(velocity_x, velocity_y, velocity_x_copy, velocity_y_copy);

	// Diffuse density according to density diffusion
	Diffuse(DIFFUSE, density_copy, density, density_diffusion);
	Advect(DIFFUSE, density, density_copy, velocity_x, velocity_y);

	// Diffuse temperature according to temperature diffusion
	Diffuse(DIFFUSE, temperature_copy, temperature, temperature_diffusion);
	Advect(DIFFUSE, temperature, temperature_copy, velocity_x, velocity_y);
}

// Render the fluid simulation to the display
void Fluid::Render(Display& display, RenderMode render_mode)
{
	const float scale_x = display.GetBufferWidth()  / (float)size;
	const float scale_y = display.GetBufferHeight() / (float)size;

	vec3* screen    = display.GetBuffer();
	const int width = display.GetBufferWidth();

	for (int j = 0; j < size; j++)
	{
		for (int i = 0; i < size; i++)
		{
			if (obstacle[INDEX(i, j)])
			{
				screen[(int)(i * scale_x + j * scale_y * width)] = vec3(0.6f, 0.2f, 0.2f);
			}
			else
			{
				switch (render_mode)
				{
					case RenderMode::RENDER_DENSITY:
					{
						screen[(int)(i * scale_x + j * scale_y * width)] = vec3(density[INDEX(i, j)]);

						break;
					}

					case RenderMode::RENDER_TEMPERATURE:
					{
						const int temp = CLAMP((int)temperature[INDEX(i, j)], 0, palette_size - 1);

						screen[(int)(i * scale_x + j * scale_y * width)] = palette[temp];

						break;
					}

					case RenderMode::RENDER_VELOCITY:
					{
						screen[(int)(i * scale_x + j * scale_y * width)] = vec3(velocity_x[INDEX(i, j)] + 0.5f, velocity_y[INDEX(i, j)] + 0.5f, 0);

						break;
					}
				}
			}
		}
	}
}

void Fluid::Reset()
{
	for (int i = 0; i < size * size; i++)
	{
		density[i] = 0;
		density_copy[i] = 0;

		temperature[i] = room_temperature;
		temperature_copy[i] = room_temperature;

		velocity_x[i] = 0;
		velocity_x_copy[i] = 0;

		velocity_y[i] = 0;
		velocity_y_copy[i] = 0;

		obstacle[i] = false;
	}
}

void Fluid::Diffuse(const Boundary b, float* x, const float* x0, const float amount)
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
			// Compute divergence
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
	
	const float edge         = size + 0.5f;
	const float size_minus_1 = size - 1;

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
			const float i0 = floorf(x);
			const float j0 = floorf(y);

			// Check if the indices are valid
			// We want to ensure that i0, j0, i1, j1 < size - 1
			// This means we only need to check when i0, j0 >= 255 because i1 = i0 + 1 and j1 = j0 + 1
			if (i0 >= size_minus_1 || j0 >= size_minus_1) continue;

			const float i1 = i0 + 1.0f;
			const float j1 = j0 + 1.0f;

			// Find interpolation factors
			const float s1 = x - i0;
			const float s0 = 1 - s1;
			const float t1 = y - j0;
			const float t0 = 1 - t1;
			
			// Cast to int for indexing
			const int i0i = (int)i0;
			const int i1i = (int)i1;
			const int j0i = (int)j0;
			const int j1i = (int)j1;

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
						const int index_left  = INDEX(i - 1, j);
						const int index_right = INDEX(i + 1, j);

						if (!obstacle[index_left])
						{
							// If the cell to the left is free, negate x component of velocity
							x[ij] = -x[index_left];
						}
						else if (!obstacle[index_right])
						{
							// If the cell to the right is free, negate x component of velocity
							x[ij] = -x[index_right];
						}
						else
						{
							// Averge of left and right cells
							x[ij] = 0.5f * (x[index_left] + x[index_right]);
						}

						break;
					}

					case Boundary::VELOCITY_Y:
					{
						const int index_up   = INDEX(i, j - 1);
						const int index_down = INDEX(i, j + 1);

						if (!obstacle[index_up])
						{
							// If the cell above is free, negate y component of velocity
							x[ij] = -x[index_up];
						}
						else if (!obstacle[index_down])
						{
							// If the cell below is free, negate y component of velocity
							x[ij] = -x[index_down];
						}
						else
						{
							// Average of up and down cells
							x[ij] = 0.5f * (x[index_up] + x[index_down]);
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

// Resolve external forces
void Fluid::ExternalForces()
{
	const float kappa =  1.1f; // Gravity and Mass scale factor (downward force)
	const float sigma = -5.0f; // Temperature scale factor		(upward   force)

	const float inv_room_temp = 1.0f / room_temperature;
	
	for (int j = 1; j < size - 1; j++)
	{
		for (int i = 1; i < size - 1; i++)
		{
			const int ij = INDEX(i, j);

			// external force = gravity + buoyancy
			const float f_ext = kappa * density[ij] + sigma * (inv_room_temp - 1.0f / temperature[ij]);

			// Apply buoyancy force in the up direction
			velocity_y[ij] += f_ext * delta_time;
		}
	}

	SetBound(Boundary::VELOCITY_Y, velocity_y);
}

// Gauss-Seidel method to iteratively solve linear systems
// x is what we are trying to solve for
void Fluid::GaussSeidel(const Boundary b, float* x, const float* x0, const float a, const float c)
{
	const float inv_c = 1.0f / c;

	for (int k = 0; k < ITERATIONS; k++)
	{
		for (int j = 1; j < size - 1; j++) 
		{
			for (int i = 1; i < size - 1; i++) 
			{
				x[INDEX(i, j)] = inv_c * (x0[INDEX(i, j)] + a * (x[INDEX(i + 1, j    )] +
												                 x[INDEX(i - 1, j    )] +
												                 x[INDEX(i,     j + 1)] +
												                 x[INDEX(i,     j - 1)]));
			}
		}

		SetBound(b, x);
	}
}