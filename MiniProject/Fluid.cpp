#include <math.h>
#include <stdlib.h>
#include <glm/glm.hpp>
#include "Fluid.h"

// Number of iterations used by Gauss-Seidel
#define ITERATIONS 4

// Get the 1d index from two 2d indices
#define INDEX(x, y) ((x) + (y) * size)

// FOR DEBUG PURPOSES
// int INDEX(int x, int y)
// {
// 	if (x < 0 || x >= 256) throw new std::exception();
// 	if (y < 0 || y >= 256) throw new std::exception();
// 
// 	return x + y * 256;
// }

// Clamps val between min and max
#define CLAMP(val, min, max) (val < min ? min : (val > max ? max : val))

using namespace std;
using namespace glm;

Fluid::Fluid(int size, float dt, int diffusion, int viscosity) : size(size), dt(dt), diff(diffusion), visc(viscosity)
{
	// Allocate grids
	const int mem_size = size * size;

	density      = (float*)calloc(mem_size, sizeof(float));
	density_prev = (float*)calloc(mem_size, sizeof(float));

	velocity_x = (float*)calloc(mem_size, sizeof(float));
	velocity_y = (float*)calloc(mem_size, sizeof(float));

	velocity_x_prev = (float*)calloc(mem_size, sizeof(float));
	velocity_y_prev = (float*)calloc(mem_size, sizeof(float));
}

Fluid::~Fluid()
{
	delete[] density;
	delete[] density_prev;

	delete[] velocity_x;
	delete[] velocity_x_prev;

	delete[] velocity_y;
	delete[] velocity_y_prev;
}

void Fluid::AddDensity(int x, int y, float amount)
{
	density[INDEX(x, y)] += amount;
}

void Fluid::AddVelocity(int x, int y, float amountX, float amountY)
{
	const int index = INDEX(x, y);

	velocity_x[index] += amountX;
	velocity_y[index] += amountY;
}

void Fluid::Step()
{
	// Diffuse the velocity field according to viscosity
	Diffuse(1, velocity_x_prev, velocity_x, visc, dt);
	Diffuse(2, velocity_y_prev, velocity_y, visc, dt);

	// Force the velocities to be mass conserving
	Project(velocity_x_prev, velocity_y_prev, velocity_x, velocity_y);

	Advect(1, velocity_x, velocity_x_prev, velocity_x_prev, velocity_y_prev, dt);
	Advect(2, velocity_y, velocity_y_prev, velocity_x_prev, velocity_y_prev, dt);

	Project(velocity_x, velocity_y, velocity_x_prev, velocity_y_prev);

	Diffuse(0, density_prev, density, diff, dt);
	Advect(0, density, density_prev, velocity_x, velocity_y, dt);
}

void Fluid::Render(Display& display)
{
	const float scale_x = display.GetBufferWidth()  / (float)size;
	const float scale_y = display.GetBufferHeight() / (float)size;

	vec3* screen = display.GetBuffer();

	for (int j = 1; j < size - 1; j++)
	{
		for (int i = 1; i < size - 1; i++)
		{
			screen[(int)(i * scale_x + j * scale_y * display.GetBufferWidth())] = vec3(density[INDEX(i, j)]);
		}
	}
}

void Fluid::Diffuse(const int b, float* x, float* x0, const float amount, const float dt)
{
	const float a = dt * amount * (size - 2) * (size - 2);

	GaussSeidel(b, x, x0, a, 1 + 6 * a);
}

// Every velocity field is the sum of an incompressible field and a gradient field
// To obtain an incompressible field we subtract the gradient field from our current velocities
void Fluid::Project(float* velocX, float* velocY, float* p, float* div)
{
	for (int j = 1; j < size - 1; j++) 
	{
		for (int i = 1; i < size - 1; i++) 
		{
			div[INDEX(i, j)] = -0.5f * (velocX[INDEX(i + 1, j    )] -
									    velocX[INDEX(i - 1, j    )] +
									    velocY[INDEX(i,     j + 1)] -
								        velocY[INDEX(i,     j - 1)]) / size;
			p[INDEX(i, j)] = 0;
		}
	}

	SetBound(0, div);
	SetBound(0, p);

	GaussSeidel(0, p, div, 1, 6);

	for (int j = 1; j < size - 1; j++) 
	{
		for (int i = 1; i < size - 1; i++) 
		{
			velocX[INDEX(i, j)] -= 0.5f * (p[INDEX(i + 1, j    )] - p[INDEX(i - 1, j    )]) * size;
			velocY[INDEX(i, j)] -= 0.5f * (p[INDEX(i,     j + 1)] - p[INDEX(i,     j - 1)]) * size;
		}
	}

	SetBound(1, velocX);
	SetBound(2, velocY);
}

void Fluid::Advect(const int b, float* d, float* d0, float* velocX, float* velocY, const float dt)
{
	const float dtx = dt * (size - 2);
	const float dty = dt * (size - 2);
	
	const float edge = size - 1.5f;

	for (int j = 1; j < size - 1; j++) 
	{
		for (int i = 1; i < size - 1; i++) 
		{
			// Move backward according to current velocities
			float x = i - dtx * velocX[INDEX(i, j)];
			float y = j - dty * velocY[INDEX(i, j)];

			x = CLAMP(x, 0.5f, edge);
			y = CLAMP(y, 0.5f, edge);

			// Find nearest corners
			float i0 = floorf(x);
			float j0 = floorf(y);
			float i1 = i0 + 1.0f;
			float j1 = j0 + 1.0f;

			// Find bilinear factors
			float s1 = x - i0;
			float s0 = 1.0f - s1;
			float t1 = y - j0;
			float t0 = 1.0f - t1;
			 
			// Cast to int for indexing
			int i0i = i0;
			int i1i = i1;
			int j0i = j0;
			int j1i = j1;

			d[INDEX(i, j)] =
				s0 * (t0 * d0[INDEX(i0i, j0i)] + t1 * d0[INDEX(i0i, j1i)]) +
				s1 * (t0 * d0[INDEX(i1i, j0i)] + t1 * d0[INDEX(i1i, j1i)]);
		}
	}

	SetBound(b, d);
}

void Fluid::SetBound(const int b, float* x)
{
	// Handle top and bottom boundaries
	for (int k = 1; k < size - 1; k++)
	{
		for (int i = 1; i < size - 1; i++)
		{
			x[INDEX(i, 0       )] = b == 2 ? -x[INDEX(i, 1       )] : x[INDEX(i, 1       )];
			x[INDEX(i, size - 1)] = b == 2 ? -x[INDEX(i, size - 2)] : x[INDEX(i, size - 2)];
		}
	}

	// Handle left and right boundaries
	for (int k = 1; k < size - 1; k++)
	{
		for (int j = 1; j < size - 1; j++)
		{
			x[INDEX(0,        j)] = b == 1 ? -x[INDEX(1,        j)] : x[INDEX(1,        j)];
			x[INDEX(size - 1, j)] = b == 1 ? -x[INDEX(size - 2, j)] : x[INDEX(size - 2, j)];
		}
	}

	// Handle the corners of the boundary
	x[INDEX(0,        0       )] = 0.3333f * (x[INDEX(1,        0       )] + x[INDEX(0,        1       )] + x[INDEX(0,        0       )]);
	x[INDEX(0,        size - 1)] = 0.3333f * (x[INDEX(1,        size - 1)] + x[INDEX(0,        size - 2)] + x[INDEX(0,        size - 1)]);
	x[INDEX(0,        0       )] = 0.3333f * (x[INDEX(1,        0       )] + x[INDEX(0,        1       )] + x[INDEX(0,        0       )]);
	x[INDEX(0,        size - 1)] = 0.3333f * (x[INDEX(1,        size - 1)] + x[INDEX(0,        size - 2)] + x[INDEX(0,        size - 1)]);
	x[INDEX(size - 1, 0       )] = 0.3333f * (x[INDEX(size - 2, 0       )] + x[INDEX(size - 1, 1       )] + x[INDEX(size - 1, 0       )]);
	x[INDEX(size - 1, size - 1)] = 0.3333f * (x[INDEX(size - 2, size - 1)] + x[INDEX(size - 1, size - 2)] + x[INDEX(size - 1, size - 1)]);
	x[INDEX(size - 1, 0       )] = 0.3333f * (x[INDEX(size - 2, 0       )] + x[INDEX(size - 1, 1       )] + x[INDEX(size - 1, 0       )]);
	x[INDEX(size - 1, size - 1)] = 0.3333f * (x[INDEX(size - 2, size - 1)] + x[INDEX(size - 1, size - 2)] + x[INDEX(size - 1, size - 1)]);
}

// Gauss-Seidel method to iteratively solve a linear system
// x is what we are trying to solve for
void Fluid::GaussSeidel(const int b, float* x, float* x0, const float a, const float c)
{
	const float inv_c = 1.0 / c;

	for (int k = 0; k < ITERATIONS; k++)
	{
		for (int j = 1; j < size - 1; j++) 
		{
			for (int i = 1; i < size - 1; i++) 
			{
				x[INDEX(i, j)] = (x0[INDEX(i, j)]+ a * (x[INDEX(i + 1, j    )] +
												        x[INDEX(i - 1, j    )] +
												        x[INDEX(i,     j + 1)] +
												        x[INDEX(i,     j - 1)] +
												        x[INDEX(i,     j    )] +
												        x[INDEX(i,     j    )])) * inv_c;
			}
		}

		SetBound(b, x);
	}
}