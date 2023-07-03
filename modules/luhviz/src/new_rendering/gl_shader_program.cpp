#include "include/gl_shader_program.hpp"
#include <GLFW/glfw3.h>

namespace luhsoccer::luhviz {

void GLShaderProgram::create(const std::string &vertex_shader_path, const std::string &fragment_shader_path) {
    // error logging
    GLint result = GL_FALSE;
    int info_log_length = 0;

    this->id = glCreateProgram();

    // create vertex and fragment shader
    GLShader vertex_shader;
    vertex_shader.create(vertex_shader_path, true);
    GLShader fragment_shader;
    fragment_shader.create(fragment_shader_path, false);

    // Link the program
    LOG_DEBUG(logger, "Linking program");
    glAttachShader(this->id, vertex_shader.getId());
    glAttachShader(this->id, fragment_shader.getId());
    glLinkProgram(this->id);

    // Check the program
    glGetProgramiv(this->id, GL_LINK_STATUS, &result);
    glGetProgramiv(this->id, GL_INFO_LOG_LENGTH, &info_log_length);
    if (info_log_length > 0) {
        std::vector<char> program_error_message(info_log_length + 1);
        glGetProgramInfoLog(this->id, info_log_length, nullptr, &program_error_message[0]);
        LOG_ERROR(logger, "{}", &program_error_message[0]);
    }

    // detach shaders
    glDetachShader(this->id, vertex_shader.getId());
    glDetachShader(this->id, fragment_shader.getId());

    this->created = true;
}

GLShaderProgram::~GLShaderProgram() {
    if (created) {
        glDeleteProgram(this->id);
    }
}

GLuint GLShaderProgram::getId() { return this->id; }

}  // namespace luhsoccer::luhviz