#pragma once
#include "Display.h"

// Array type used by boundary condition
enum Boundary
{
	DIFFUSE    = 0,
	VELOCITY_X = 1,
	VELOCITY_Y = 2
};

class Fluid
{
public:
	const int size;
	const float delta_time;
	const float diffusion;
	const float viscosity;
	const float room_temperature;

	float* density;
	float* density_prev;

	float* temperature;
	float* temperature_prev;

	float* velocity_x;
	float* velocity_x_prev;

	float* velocity_y;
	float* velocity_y_prev;

	bool* obstacle;

	Fluid(int size, float delta_time, float diffusion, float viscosity, float room_temperature);
	~Fluid();

	void AddDensity(int x, int y, float amount);
	void AddTemperature(int x, int y, float amount);
	void AddVelocity(int x, int y, float amountX, float amountY);
	void AddObstacle(int x, int y);
	void RemoveObstacle(int x, int y);

	void Update();
	void Render(Display& display);

	void Diffuse(const Boundary b, float* x, float* x0, const float amount);
	void Advect(const Boundary b, float* d, float* d0, const float* vel_x, const float* vel_y);
	void Project(float* vel_x, float* vel_y, float* p, float* div);
	void SetBound(const Boundary b, float* x);

	void Buoyancy();

	void GaussSeidel(const Boundary b, float* x, float* x0, const float a, const float c);
};
