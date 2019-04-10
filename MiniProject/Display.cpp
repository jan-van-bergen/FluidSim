#include "Display.h"

#include <string>
#include <cstring>
#include <iostream>

#include <Imgui/imgui.h>
#include <Imgui/imgui_impl_sdl.h>
#include <Imgui/imgui_impl_opengl3.h>

using namespace glm;
using namespace std;

Display::Display(int window_width, int window_height, int buffer_width, int buffer_height, const string& title)
{
	m_window_width = window_width;
	m_window_height = window_height;

	m_buffer_width = buffer_width;
	m_buffer_height = buffer_height;

	// Create SDL window
	SDL_Init(SDL_INIT_EVERYTHING);

	SDL_GL_SetAttribute(SDL_GL_RED_SIZE, 8);
	SDL_GL_SetAttribute(SDL_GL_GREEN_SIZE, 8);
	SDL_GL_SetAttribute(SDL_GL_BLUE_SIZE, 8);
	SDL_GL_SetAttribute(SDL_GL_ALPHA_SIZE, 8);
	SDL_GL_SetAttribute(SDL_GL_BUFFER_SIZE, 32);
	SDL_GL_SetAttribute(SDL_GL_DOUBLEBUFFER, 1);

	m_window = SDL_CreateWindow(title.c_str(), SDL_WINDOWPOS_CENTERED, SDL_WINDOWPOS_CENTERED, window_width, window_height, SDL_WINDOW_OPENGL);
	m_glContext = SDL_GL_CreateContext(m_window);

	// Initialize OpenGL
	GLenum status = glewInit();

	if (status != GLEW_OK)
	{
		cerr << "Glew failed to initialize!" << endl;
	}

	m_isClosed = false;

	glEnable(GL_TEXTURE_2D);
	glDisable(GL_DEPTH_TEST);
	glHint(GL_PERSPECTIVE_CORRECTION_HINT, GL_FASTEST);

	glViewport(0, 0, window_width, window_height);
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	glOrtho(-1.0f, 1.0f, -1.0f, 1.0f, 0.0f, 4.0f);

	m_pixels = new vec3[buffer_width * buffer_height];

	glGenTextures(1, &m_screen);
	glBindTexture(GL_TEXTURE_2D, m_screen);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
	glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, buffer_width, buffer_height, 0, GL_RGB, GL_FLOAT, m_pixels);
	glBindTexture(GL_TEXTURE_2D, 0);

	// Setup camera
	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();

	// Initialize IMGUI
	IMGUI_CHECKVERSION();
	ImGui::CreateContext();

	// Setup Platform/Renderer bindings
	ImGui_ImplSDL2_InitForOpenGL(m_window, m_glContext);
	ImGui_ImplOpenGL3_Init("#version 130");

	ImGui::StyleColorsDark();
}

Display::~Display()
{
	// Free framebuffer
	delete[] m_pixels;

	// Clean up IMGUI
	ImGui_ImplOpenGL3_Shutdown();
	ImGui_ImplSDL2_Shutdown();
	ImGui::DestroyContext();

	// Cleanup SDL
	SDL_GL_DeleteContext(m_glContext);
	SDL_DestroyWindow(m_window);
	SDL_Quit();
}

void Display::SetTitle(const std::string& title)
{
	SDL_SetWindowTitle(m_window, title.c_str());
}

void Display::Clear()
{
	for (int j = 0; j < m_buffer_height; j++)
	{
		for (int i = 0; i < m_buffer_width; i++)
		{
			Plot(i, j, 0, 0, 0);
		}
	}
}

void Display::StartGUI()
{
	ImGui_ImplOpenGL3_NewFrame();
	ImGui_ImplSDL2_NewFrame(m_window);
	ImGui::NewFrame();
}

void Display::Update()
{
	// Clear window contents
	glClear(GL_COLOR_BUFFER_BIT);

	glBindTexture(GL_TEXTURE_2D, m_screen);
	glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, m_buffer_width, m_buffer_height, 0, GL_RGB, GL_FLOAT, m_pixels);

	// Draw screen filling quad
	glBegin(GL_QUADS);
	glTexCoord2f(0.0f, 1.0f); glVertex2f(-1.0f, -1.0f);
	glTexCoord2f(1.0f, 1.0f); glVertex2f(1.0f, -1.0f);
	glTexCoord2f(1.0f, 0.0f); glVertex2f(1.0f, 1.0f);
	glTexCoord2f(0.0f, 0.0f); glVertex2f(-1.0f, 1.0f);
	glEnd();

	// Render IMGUI
	ImGui::Render();
	ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());

	// Swap buffers to present rendered image
	SDL_GL_SwapWindow(m_window);
}