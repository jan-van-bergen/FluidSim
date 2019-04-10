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

Fluid::Fluid(int size, float delta_time, float viscosity, float density_diffusion, float temperature_diffusion, float room_temperature, float vorticity_confinement_eta) :
	size(size), 
	viscosity(viscosity), 
	delta_time(delta_time), 
	density_diffusion(density_diffusion),
	temperature_diffusion(temperature_diffusion),
	room_temperature(room_temperature),
	vorticity_confinement(vorticity_confinement_eta)
{
	kappa =  0.1f;
	sigma = -0.2f;

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
	SDL_Surface* palette_bmp = SDL_LoadBMP("../MiniProject/Data/Heat_palette.bmp");

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
	// Free memory
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

// Add the given amount of density at the given x,y coordinate
void Fluid::AddDensity(int x, int y, float amount)
{
	const int index = INDEX(x, y);

	if (obstacle[index]) return;

	density[index] += amount;
}

// Add the given amount of velocity at the given x,y coordinate
void Fluid::AddVelocity(int x, int y, float amountX, float amountY)
{
	const int index = INDEX(x, y);

	if (obstacle[index]) return;

	velocity_x[index] += amountX;
	velocity_y[index] += amountY;
}

// Add the given amount of temperature at the given x,y coordinate
void Fluid::AddTemperature(int x, int y, float amount)
{
	const int index = INDEX(x, y);

	if (obstacle[index]) return;

	temperature[index] += amount;
}

// Adds an obstacle in the region around the given x,y coordinate
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
	ExternalForces(velocity_x, velocity_y);

	// Apply velocity confinement (basically also an external force)
	VorticityConfinement(velocity_x, velocity_y, velocity_x_copy);

	// Project to incompressible fluid
	Project(velocity_x, velocity_y, velocity_x_copy, velocity_y_copy);

	// Diffuse density according to density diffusion
	Diffuse(OTHER, density_copy, density, density_diffusion);
	Advect(OTHER, density, density_copy, velocity_x, velocity_y);

	// Diffuse temperature according to temperature diffusion
	Diffuse(OTHER, temperature_copy, temperature, temperature_diffusion);
	Advect(OTHER, temperature, temperature_copy, velocity_x, velocity_y);
}

// Render the fluid simulation to the display
void Fluid::Render(Display& display, RenderMode render_mode)
{
	const float scale_x = display.GetBufferWidth()  / (float)size;
	const float scale_y = display.GetBufferHeight() / (float)size;

	vec3* screen    = display.GetBuffer();
	const int width = display.GetBufferWidth();

#pragma omp parallel for
	for (int j = 0; j < size; j++)
	{
		for (int i = 0; i < size; i++)
		{
			if (obstacle[INDEX(i, j)])
			{
				// Draw grey pixel to indicate obstacle
				screen[(int)(i * scale_x + j * scale_y * width)] = vec3(0.3f, 0.3f, 0.3f);
			}
			else
			{
				vec3 colour;

				switch (render_mode)
				{
					case RenderMode::RENDER_DENSITY:
					{
						// Multiply density by 2.5f for better visual clarity
						colour = vec3(density[INDEX(i, j)] * 2.5f);

						break;
					}

					case RenderMode::RENDER_TEMPERATURE:
					{
						// Sample heat map palette
						colour = palette[CLAMP((int)temperature[INDEX(i, j)], 0, palette_size - 1)];

						break;
					}

					case RenderMode::RENDER_VELOCITY:
					{
						colour = vec3(velocity_x[INDEX(i, j)] + 0.5f, velocity_y[INDEX(i, j)] + 0.5f, 0.5f);

						break;
					}
				}
				
				screen[(int)(i * scale_x + j * scale_y * width)] = colour;
			}
		}
	}
}

// Reset all quanitites in the grid to their defaults
void Fluid::Reset()
{
#pragma omp parallel for
	for (int i = 0; i < size * size; i++)
	{
		density[i]      = 0;
		density_copy[i] = 0;

		temperature[i]      = room_temperature;
		temperature_copy[i] = room_temperature;

		velocity_x[i]      = 0;
		velocity_x_copy[i] = 0;

		velocity_y[i]      = 0;
		velocity_y_copy[i] = 0;

		obstacle[i] = false;
	}
}

// Diffuse field x0 and store it in x
void Fluid::Diffuse(const BoundaryType b, float* x, const float* x0, const float amount)
{
	const float a = delta_time * amount * (size - 2) * (size - 2);

	GaussSeidel(b, x, x0, a, 1 + 4 * a);
}

// Semi-Lagrangian advection
// Advect field d0 and store it in d, according to the x and y velocities in the grid
void Fluid::Advect(const BoundaryType b, float* d, const float* d0, const float* vel_x, const float* vel_y)
{
	const float delta_size = delta_time * (size - 2);

	const float edge         = size + 0.5f;
	const float size_minus_1 = size - 1;

#pragma omp parallel for
	for (int j = 1; j < size - 1; j++) 
	{
		for (int i = 1; i < size - 1; i++) 
		{
			if (obstacle[INDEX(i, j)]) continue;

			// Move backward according to current velocities
			float x = i - delta_size * vel_x[INDEX(i, j)];
			float y = j - delta_size * vel_y[INDEX(i, j)];
			
			// Clamp x and y to be valid
			x = CLAMP(x, 0.5f, edge);
			y = CLAMP(y, 0.5f, edge);

			// Find nearest corners
			const float left = floorf(x);
			const float up   = floorf(y);

			// Check if the indices are valid
			// We want to ensure that left, up, right, down < size - 1
			// This means we only need to check when left, up >= 255 because right = left + 1 and down = up + 1
			if (left >= size_minus_1 || up >= size_minus_1) continue;

			const float right = left + 1.0f;
			const float down  = up   + 1.0f;

			// Find interpolation factors
			const float s1 = x - left;
			const float s0 = 1 - s1;
			const float t1 = y - up;
			const float t0 = 1 - t1;
			
			// Cast to int for indexing
			const int i0 = (int)left;
			const int i1 = (int)right;
			const int j0 = (int)up;
			const int j1 = (int)down;

			d[INDEX(i, j)] =
				s0 * (t0 * d0[INDEX(i0, j0)] + t1 * d0[INDEX(i0, j1)]) +
				s1 * (t0 * d0[INDEX(i1, j0)] + t1 * d0[INDEX(i1, j1)]);
		}
	}

	// Enforce boundary conditions on the advected field
	BoundaryConditions(b, d);
}

// Resolve external forces
void Fluid::ExternalForces(float* vel_x, float* vel_y)
{
	const float inv_room_temp = 1.0f / room_temperature;
	
#pragma omp parallel for
	for (int j = 1; j < size - 1; j++)
	{
		for (int i = 1; i < size - 1; i++)
		{
			const int ij = INDEX(i, j);

			if (obstacle[ij]) continue;

			// external force = gravity + buoyancy
			const float f_ext = kappa * density[ij] + sigma * (inv_room_temp - 1.0f / temperature[ij]);

			// Apply buoyancy force in the up direction
			vel_y[ij] += f_ext * delta_time;
		}
	}

	BoundaryConditions(BoundaryType::VELOCITY_Y, vel_y);
}

// Apply vorticity confinement to recover some of the energy lost due to numerical dissipation
// This allows the fluid to exhibit small scale curl, which it otherwise could not
// Stores vorticity/curl in curl
void Fluid::VorticityConfinement(float* vel_x, float* vel_y, float* curl)
{
	// Compute curl
#pragma omp parallel for
	for (int j = 1; j < size - 1; j++)
	{
		for (int i = 1; i < size - 1; i++)
		{			
			curl[INDEX(i, j)] = vel_x[INDEX(i,     j + 1)] - vel_x[INDEX(i,     j - 1)] +
				                vel_y[INDEX(i - 1, j    )] - vel_y[INDEX(i + 1, j    )];
		}
	}

	// Apply vorticity confinement based on the curl
#pragma omp parallel for
	for (int j = 1; j < size - 1; j++)
	{
		for (int i = 1; i < size - 1; i++)
		{
			const int ij = INDEX(i, j);

			if (obstacle[ij]) continue;
			
			// Compute gradient of the absolute value of the curl in x and y directions
			const float dx = abs(curl[INDEX(i + 1, j    )]) - abs(curl[INDEX(i - 1, j    )]);
			const float dy = abs(curl[INDEX(i,     j - 1)]) - abs(curl[INDEX(i    , j + 1)]);

			// Normalization factor for dx and dy (1e-5 avoids zero division)
			const float inv_length = 1.0f / (sqrtf(dx * dx + dy * dy) + 1e-5);

			vel_x[ij] += delta_time * curl[ij] * vorticity_confinement * inv_length * dy;
			vel_y[ij] += delta_time * curl[ij] * vorticity_confinement * inv_length * dx;
		}
	}
}

// Every velocity field is the sum of an incompressible field and a gradient field
// To obtain an incompressible field we subtract the gradient field from our current velocities
// Stores pressure in p and divergence in div (both are used as temporary buffers)
void Fluid::Project(float* vel_x, float* vel_y, float* p, float* div)
{
	const float factor = -0.5f / size;

#pragma omp parallel for
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

	// Enforce boundary conditions
	BoundaryConditions(OTHER, div);
	BoundaryConditions(OTHER, p);

	// Obtain pressure from diffusing divergence
	GaussSeidel(OTHER, p, div, 1, 4);

#pragma omp parallel for
	for (int j = 1; j < size - 1; j++)
	{
		for (int i = 1; i < size - 1; i++)
		{
			const int ij = INDEX(i, j);

			if (obstacle[ij]) continue;

			// Subtract pressure gradient to make x and y velocities incompressible again
			vel_x[ij] -= 0.5f * size * (p[INDEX(i + 1, j    )] - p[INDEX(i - 1, j    )]);
			vel_y[ij] -= 0.5f * size * (p[INDEX(i,     j + 1)] - p[INDEX(i,     j - 1)]);
		}
	}

	// Enforce boundary conditions
	BoundaryConditions(VELOCITY_X, vel_x);
	BoundaryConditions(VELOCITY_Y, vel_y);
}

// Enforce boundary conditions on field x
void Fluid::BoundaryConditions(const BoundaryType b, float* x)
{
	// Handle edge of screen boundaries
#pragma omp parallel for
	for (int i = 1; i < size - 1; i++)
	{
		// Copy if x is not velocity_y or velocity_y prev, otherwise negate
		x[INDEX(i, 0)]        = b == BoundaryType::VELOCITY_Y ? -x[INDEX(i, 1       )] : x[INDEX(i, 1       )];
		x[INDEX(i, size - 1)] = b == BoundaryType::VELOCITY_Y ? -x[INDEX(i, size - 2)] : x[INDEX(i, size - 2)];

		// Copy if x is not velocity_x or velocity_x prev, otherwise negate
		x[INDEX(0, i       )] = b == BoundaryType::VELOCITY_X ? -x[INDEX(1, i       )] : x[INDEX(1, i       )];
		x[INDEX(size - 1, i)] = b == BoundaryType::VELOCITY_X ? -x[INDEX(size - 2, i)] : x[INDEX(size - 2, i)];
	}

	// Handle the corners of the boundary
	x[INDEX(0,        0)]        = 0.5f * (x[INDEX(1,        0)]        + x[INDEX(0,        1       )]);
	x[INDEX(0,        size - 1)] = 0.5f * (x[INDEX(1,        size - 1)] + x[INDEX(0,        size - 2)]);
	x[INDEX(size - 1, 0)]        = 0.5f * (x[INDEX(size - 2, 0)]        + x[INDEX(size - 1, 1       )]);
	x[INDEX(size - 1, size - 1)] = 0.5f * (x[INDEX(size - 2, size - 1)] + x[INDEX(size - 1, size - 2)]);

	// Lookup table for the inverse of the numbers 1, 2, 3, 4 
	// We don't care about zero division so we arbitrarily define 1/0 = 1
	const float inv[5] = { 1.0f, 1.0f, 0.5f, 0.3333333f, 0.25f };

	// Handle obstacles in the grid
#pragma omp parallel for
	for (int j = 1; j < size - 1; j++)
	{
		for (int i = 1; i < size - 1; i++)
		{
			const int ij = INDEX(i, j);

			if (obstacle[ij])
			{
				switch (b)
				{
					case BoundaryType::VELOCITY_X:
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

					case BoundaryType::VELOCITY_Y:
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

					case BoundaryType::OTHER:
					{
						// Compute average of all direct neightbours that are not obstacles
						float sum = 0;
						int count = 0;
						
						if (!obstacle[INDEX(i - 1, j    )]) { sum += x[INDEX(i - 1, j    )]; count++; } else // Wallthickness is always >= 2, so if left is free, right must contain an obstacle
						if (!obstacle[INDEX(i + 1, j    )]) { sum += x[INDEX(i + 1, j    )]; count++; }
						if (!obstacle[INDEX(i,     j - 1)]) { sum += x[INDEX(i,     j - 1)]; count++; } else // Wallthickness is always >= 2, so if above is free, down must contain an obstacle
						if (!obstacle[INDEX(i,     j + 1)]) { sum += x[INDEX(i,     j + 1)]; count++; }

						// Avoid zero division
						if (count > 0)
						{
							// Divide sum by count using lookup table
							x[ij] = sum * inv[count];
						}
					}
				}
			}
		}
	}
}

// Gauss-Seidel method to iteratively solve linear systems
// x is what we are trying to solve for
void Fluid::GaussSeidel(const BoundaryType b, float* x, const float* x0, const float a, const float c)
{
	const float inv_c = 1.0f / c;

	for (int k = 0; k < ITERATIONS; k++)
	{
#pragma omp parallel for
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

		BoundaryConditions(b, x);
	}
}