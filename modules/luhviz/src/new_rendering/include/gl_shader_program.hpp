#pragma once

#include <glad/glad.h>
#include "logger/logger.hpp"
#include "new_rendering/include/gl_shader.hpp"

namespace luhsoccer::luhviz {

class GLShaderProgram {
   public:
    GLShaderProgram(const GLShaderProgram&) = default;
    GLShaderProgram(GLShaderProgram&&) = delete;
    GLShaderProgram& operator=(const GLShaderProgram&) = default;
    GLShaderProgram& operator=(GLShaderProgram&&) = default;

    GLShaderProgram() = default;
    ~GLShaderProgram();

    void create(const std::string& vertex_shader_path, const std::string& fragment_shader_path);
    GLuint getId();

   private:
    logger::Logger logger{"Luhviz/GLShaderProgram"};

    GLuint id{};
    bool created{false};
};

}  // namespace luhsoccer::luhviz