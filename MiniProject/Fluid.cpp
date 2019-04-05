#include <math.h>
#include <stdlib.h>
#include <glm/glm.hpp>
#include "Fluid.h"

#define IX(x, y) ((x) + (y) * N)

//inline int IX(int x, int y)
//{
//	if (x < 0) x = 0;
//	if (x >= 256) x = 255;
//	if (y < 0) y = 0;
//	if (y >= 256) y = 255;
//
//	return x + y * 256;
//}

//inline int IX(int x, int y)
//{
//	if (x < 0 || x >= 256) throw new std::exception();
//	if (y < 0 || y >= 256) throw new std::exception();
//
//	return x + y * 256;
//}

using namespace std;
using namespace glm;

Fluid::Fluid(int size, float dt, int diffusion, int viscosity) : size(size), dt(dt), diff(diffusion), visc(viscosity)
{
	const int mem_size = size * size;

	s       = (float*)calloc(mem_size, sizeof(float));
	density = (float*)calloc(mem_size, sizeof(float));

	Vx = (float*)calloc(mem_size, sizeof(float));
	Vy = (float*)calloc(mem_size, sizeof(float));

	Vx0 = (float*)calloc(mem_size, sizeof(float));
	Vy0 = (float*)calloc(mem_size, sizeof(float));
}

Fluid::~Fluid()
{
	delete[] s;
	delete[] density;

	delete[] Vx;
	delete[] Vy;

	delete[] Vx0;
	delete[] Vy0;
}

void Fluid::AddDensity(int x, int y, float amount)
{
	const int N = size;
	density[IX(x, y)] += amount;
}

void Fluid::AddVelocity(int x, int y, float amountX, float amountY)
{
	const int N = size;
	const int index = IX(x, y);

	Vx[index] += amountX;
	Vy[index] += amountY;
}

void Fluid::Step()
{
	const int N = size;

	Diffuse(1, Vx0, Vx, visc, dt, 4, N);
	Diffuse(2, Vy0, Vy, visc, dt, 4, N);

	Project(Vx0, Vy0, Vx, Vy, 4, N);

	Advect(1, Vx, Vx0, Vx0, Vy0, dt, N);
	Advect(2, Vy, Vy0, Vx0, Vy0, dt, N);

	Project(Vx, Vy, Vx0, Vy0, 4, N);

	Diffuse(0, s, density, diff, dt, 4, N);
	Advect(0, density, s, Vx, Vy, dt, N);
}

void Fluid::Render(Display& display)
{
	const int N = size;

	const float scale_x = display.GetBufferWidth() / (float)N;
	const float scale_y = display.GetBufferHeight() / (float)N;

	vec3* screen = display.GetBuffer();

	for (int j = 1; j < N - 1; j++)
	{
		for (int i = 1; i < N - 1; i++)
		{
			const int index = (int)(i * scale_x + j * scale_y * display.GetBufferWidth());

			const int ij = IX(i, j);

			screen[(int)(i * scale_x + j * scale_y * display.GetBufferWidth())] = vec3(density[ij]);
		}
	}
}

void Fluid::Diffuse(int b, float *x, float *x0, float diff, float dt, int iter, int N)
{
	const float a = dt * diff * (N - 2) * (N - 2);

	LinearSolve(b, x, x0, a, 1 + 6 * a, iter, N);
}

void Fluid::Project(float *velocX, float *velocY, float *p, float *div, int iter, int N)
{
	for (int j = 1; j < N - 1; j++) 
	{
		for (int i = 1; i < N - 1; i++) 
		{
			div[IX(i, j)] = -0.5f * (velocX[IX(i + 1, j    )] -
									 velocX[IX(i - 1, j    )] +
									 velocY[IX(i,     j + 1)] -
									 velocY[IX(i,     j - 1)]) / N;
			p[IX(i, j)] = 0;
		}
	}

	SetBound(0, div, N);
	SetBound(0, p,   N);

	LinearSolve(0, p, div, 1, 6, iter, N);

	for (int j = 1; j < N - 1; j++) 
	{
		for (int i = 1; i < N - 1; i++) 
		{
			velocX[IX(i, j)] -= 0.5f * (p[IX(i + 1, j    )] - p[IX(i - 1, j    )]) * N;
			velocY[IX(i, j)] -= 0.5f * (p[IX(i,     j + 1)] - p[IX(i,     j - 1)]) * N;
		}
	}

	SetBound(1, velocX, N);
	SetBound(2, velocY, N);
}

void Fluid::Advect(int b, float *d, float *d0, float *velocX, float *velocY, float dt, int N)
{
	float i0, i1, j0, j1;
	 
	const float dtx = dt * (N - 2);
	const float dty = dt * (N - 2);
	 
	float s0, s1, t0, t1;
	float tmp1, tmp2, tmp3, x, y, z;
	 
	const float Nfloat = N;
	float ifloat, jfloat;
	int i, j;

	for (j = 1, jfloat = 1; j < N - 1; j++, jfloat++) 
	{
		for (i = 1, ifloat = 1; i < N - 1; i++, ifloat++) 
		{
			tmp1 = dtx * velocX[IX(i, j)];
			tmp2 = dty * velocY[IX(i, j)];

			x = ifloat - tmp1;
			y = jfloat - tmp2;

			if (x < 0.5f) x = 0.5f;
			if (x > Nfloat + 0.5f) x = Nfloat + 0.5f;

			i0 = floorf(x);
			i1 = i0 + 1.0f;

			if (y < 0.5f) y = 0.5f;
			if (y > Nfloat + 0.5f) y = Nfloat + 0.5f;

			j0 = floorf(y);
			j1 = j0 + 1.0f;

			s1 = x - i0;
			s0 = 1.0f - s1;
			t1 = y - j0;
			t0 = 1.0f - t1;

			int i0i = i0;
			int i1i = i1;
			int j0i = j0;
			int j1i = j1;

			// @TEMP:
#define TEST(index) (index < 0 || index >= 256)

			if (TEST(i0i) || TEST(i1i) || TEST(j0i) || TEST(j1i)) continue;

			d[IX(i, j)] =
				s0 * (t0 * d0[IX(i0i, j0i)] + t1 * d0[IX(i0i, j1i)]) +
				s1 * (t0 * d0[IX(i1i, j0i)] + t1 * d0[IX(i1i, j1i)]);
		}
	}

	SetBound(b, d, N);
}

void Fluid::SetBound(int b, float *x, int N)
{
	for (int k = 1; k < N - 1; k++)
	{
		for (int i = 1; i < N - 1; i++)
		{
			x[IX(i, 0    )] = b == 2 ? -x[IX(i, 1    )] : x[IX(i, 1    )];
			x[IX(i, N - 1)] = b == 2 ? -x[IX(i, N - 2)] : x[IX(i, N - 2)];
		}
	}
	for (int k = 1; k < N - 1; k++)
	{
		for (int j = 1; j < N - 1; j++)
		{
			x[IX(0,     j)] = b == 1 ? -x[IX(1,     j)] : x[IX(1,     j)];
			x[IX(N - 1, j)] = b == 1 ? -x[IX(N - 2, j)] : x[IX(N - 2, j)];
		}
	}

	x[IX(0,     0    )] = 0.3333f * (x[IX(1,     0    )] + x[IX(0,     1    )] + x[IX(0,     0    )]);
	x[IX(0,     N - 1)] = 0.3333f * (x[IX(1,     N - 1)] + x[IX(0,     N - 2)] + x[IX(0,     N - 1)]);
	x[IX(0,     0    )] = 0.3333f * (x[IX(1,     0    )] + x[IX(0,     1    )] + x[IX(0,     0    )]);
	x[IX(0,     N - 1)] = 0.3333f * (x[IX(1,     N - 1)] + x[IX(0,     N - 2)] + x[IX(0,     N - 1)]);
	x[IX(N - 1, 0    )] = 0.3333f * (x[IX(N - 2, 0    )] + x[IX(N - 1, 1    )] + x[IX(N - 1, 0    )]);
	x[IX(N - 1, N - 1)] = 0.3333f * (x[IX(N - 2, N - 1)] + x[IX(N - 1, N - 2)] + x[IX(N - 1, N - 1)]);
	x[IX(N - 1, 0    )] = 0.3333f * (x[IX(N - 2, 0    )] + x[IX(N - 1, 1    )] + x[IX(N - 1, 0    )]);
	x[IX(N - 1, N - 1)] = 0.3333f * (x[IX(N - 2, N - 1)] + x[IX(N - 1, N - 2)] + x[IX(N - 1, N - 1)]);
}

void Fluid::LinearSolve(int b, float *x, float *x0, float a, float c, int iter, int N)
{
	const float inv_c = 1.0 / c;

	for (int k = 0; k < iter; k++) 
	{
		for (int j = 1; j < N - 1; j++) 
		{
			for (int i = 1; i < N - 1; i++) 
			{
				x[IX(i, j)] = (x0[IX(i, j)]+ a * (x[IX(i + 1, j    )] +
												  x[IX(i - 1, j    )] +
												  x[IX(i,     j + 1)] +
												  x[IX(i,     j - 1)] +
												  x[IX(i,     j    )] +
												  x[IX(i,     j    )])) * inv_c;
			}
		}

		SetBound(b, x, N);
	}
}