#pragma once

#include <glad/glad.h>
#include "GLFW/glfw3.h"
#include "logger/logger.hpp"
#include "new_rendering/include/gl_shader_program.hpp"
#include "new_rendering/include/gl_texture.hpp"
#include "new_rendering/include/gl_vertex_array.hpp"

namespace luhsoccer::luhviz {

class GLContext {
   public:
    GLContext(const GLContext&) = default;
    GLContext(GLContext&&) = delete;
    GLContext& operator=(const GLContext&) = default;
    GLContext& operator=(GLContext&&) = delete;

    /**
     * @brief Construct a new GLContext
     *
     */
    GLContext() = default;

    /**
     * @brief creates a new gl context
     *
     */
    void create();

    /**
     * @brief Destroy the GLContext and free data
     *
     */
    ~GLContext();

    /**
     * @brief clears everything
     *
     */
    void clear();

    /**
     * @brief set the color used in clear()
     *
     * @param red
     * @param green
     * @param blue
     * @param alpha
     */
    void clearColor(GLclampf red, GLclampf green, GLclampf blue, GLclampf alpha);

    /**
     * @brief apply the shader program to use for until another is specified
     *
     * @param program
     */
    void useProgram(GLShaderProgram& program);

    /**
     * @brief set the active texture to use
     *
     * @param texture
     * @param unit
     */
    void bindTexture(GLTexture& texture, int unit);

    /**
     * @brief draw vao with indexed vertices
     *
     * @param vao
     * @param num_indices
     */
    void drawElements(GLVertexArray& vao, int num_indices);

    /**
     * @brief draw vao not indexed
     *
     * @param vao
     * @param size
     */
    void drawArrays(GLVertexArray& vao, int size);

    /**
     * @brief bind the frame buffer
     *
     */
    void bindFrameBuffer();

    /**
     * @brief enable graphics setting
     *
     * @param which
     */
    void enable(GLenum which);

    /**
     * @brief disable graphics setting
     *
     * @param which
     */
    void disable(GLenum which);

    /**
     * @brief define the function to calculate depth
     *
     * @param func
     */
    void depthFunc(GLenum func);

    /**
     * @brief define how to do blending
     *
     * @param sfactor
     * @param dfactor
     */
    void blendFunc(GLenum sfactor, GLenum dfactor);

    /**
     * @brief Get the Uniform Handle object
     *
     * @return GLint
     */
    GLint getUniformHandle();

    /**
     * @brief displays a text
     *
     */
    void displayText(float x, float y, float size, int32_t color, const std::string& text, float* projection_matrix);

    /**
     * @brief enables debugging mode for opengl 4.0+
     *
     */
    void enableDebugging(bool enable);

   private:
    logger::Logger logger{"Luhviz/GLContext"};
    bool created = false;
};

}  // namespace luhsoccer::luhviz