#pragma once
#include <glm/glm.hpp>
#include "Display.h"

// Array type used by boundary condition
enum BoundaryType
{
	VELOCITY_X = 0,
	VELOCITY_Y = 1,
	OTHER      = 2
};

// What field should we draw to the screen
enum RenderMode
{
	RENDER_DENSITY,
	RENDER_TEMPERATURE,
	RENDER_VELOCITY
};
const std::string RenderMode_Names[] = { "Density", "Temperature", "Velocity" };

class Fluid
{
private:
	const int size;
	const float delta_time;

	int palette_size;
	glm::vec3* palette;

	float* density;
	float* density_copy;

	float* temperature;
	float* temperature_copy;

	float* velocity_x;
	float* velocity_x_copy;

	float* velocity_y;
	float* velocity_y_copy;

	bool* obstacle;

	void Diffuse(const BoundaryType b, float* x, const float* x0, const float amount);
	void Advect(const BoundaryType b, float* d, float* d0, const float* vel_x, const float* vel_y);
	void ExternalForces(float* vel_x, float* vel_y);
	void VorticityConfinement(float* vel_x, float* vel_y, float* curl);
	void Project(float* vel_x, float* vel_y, float* p, float* div);
	void BoundaryConditions(const BoundaryType b, float* x);

	void GaussSeidel(const BoundaryType b, float* x, const float* x0, const float a, const float c);
public:

	float viscosity;
	float density_diffusion;
	float temperature_diffusion;

	float room_temperature;

	float kappa; // Gravity  scale factor
	float sigma; // Buoyancy scale factor

	float vorticity_confinement;

	Fluid(int size, float delta_time, float viscosity, float density_diffusion, float temperature_diffusion, float room_temperature, float vorticity_confinement_eta);
	~Fluid();

	void AddDensity(int x, int y, float amount);
	void AddTemperature(int x, int y, float amount);
	void AddVelocity(int x, int y, float amountX, float amountY);
	void AddObstacle(int x, int y);
	void RemoveObstacle(int x, int y);

	void Update();

	void Render(Display& display, RenderMode render_mode);

	void Reset();
};
