#pragma once
#include "Display.h"

class Fluid
{
public:
	const int size;
	const float dt;
	const float diff;
	const float visc;

	float* density;
	float* density_prev;

	float* velocity_x;
	float* velocity_x_prev;

	float* velocity_y;
	float* velocity_y_prev;

	Fluid(int size, float dt, int diffusion, int viscosity);
	~Fluid();

	void AddDensity(int x, int y, float amount);
	void AddVelocity(int x, int y, float amountX, float amountY);

	void Step();

	void Render(Display& display);

	void Diffuse(const int b, float* x, float* x0, const float amount, const float dt);
	void Advect(const int b, float* d, float* d0, float* velocX, float* velocY, const float dt);
	void Project(float* velocX, float* velocY, float* p, float* div);
	void SetBound(const int b, float* x);

	void GaussSeidel(const int b, float* x, float* x0, const float a, const float c);
};
