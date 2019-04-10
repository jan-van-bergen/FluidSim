#pragma once
#include <string>
#include <GL/glew.h>
#include <SDL2/SDL.h>
#include <glm/glm.hpp>

class Display
{
public:
	Display(int window_width, int window_height, int buffer_width, int buffer_height, const std::string& title);
	virtual ~Display();

	void Clear();
	void Update();

	void StartGUI();

	inline void Plot(int x, int y, float r, float g, float b) const
	{
		glm::vec3& pixel = m_pixels[x + m_buffer_width * y];
		pixel.r = r;
		pixel.g = g;
		pixel.b = b;
	}

	inline glm::vec3* GetBuffer()
	{
		return m_pixels;
	}

	inline bool IsClosed() { return m_isClosed; }

	void SetTitle(const std::string& title);

	inline int GetWindowWidth() { return m_window_width; }
	inline int GetWindowHeight() { return m_window_height; }

	inline int GetBufferWidth() { return m_buffer_width; }
	inline int GetBufferHeight() { return m_buffer_height; }
private:
	int m_window_width;
	int m_window_height;

	int m_buffer_width;
	int m_buffer_height;

	SDL_Window* m_window;
	SDL_GLContext m_glContext;

	glm::vec3* m_pixels;
	GLuint m_screen;

	bool m_isClosed;
};

