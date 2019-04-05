#pragma once
#include "Display.h"

class Fluid
{
public:
	const int size;
	const float dt;
	const float diff;
	const float visc;

	float *s;
	float *density;

	float *Vx;
	float *Vy;

	float *Vx0;
	float *Vy0;

	Fluid(int size, float dt, int diffusion, int viscosity);
	~Fluid();

	void AddDensity(int x, int y, float amount);
	void AddVelocity(int x, int y, float amountX, float amountY);

	void Step();

	void Render(Display& display);

	void Diffuse(int b, float *x, float *x0, float diff, float dt, int iter, int N);
	void Project(float *velocX, float *velocY, float *p, float *div, int iter, int N);
	void Advect(int b, float *d, float *d0, float *velocX, float *velocY, float dt, int N);
	void SetBound(int b, float *x, int N);
	void LinearSolve(int b, float *x, float *x0, float a, float c, int iter, int N);
};
