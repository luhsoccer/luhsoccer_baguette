#include "include/gl_shader.hpp"

#include <cmrc/cmrc.hpp>

CMRC_DECLARE(luhviz);

namespace luhsoccer::luhviz {

void GLShader::create(const std::string& shader_path, bool is_vertex_shader) {
    // enable error logging
    GLint result = GL_FALSE;
    int info_log_length = 0;

    // Read the shader code from the file
    auto fs = cmrc::luhviz::get_filesystem();
    auto vertex_file = fs.open(shader_path);
    std::string shader_code{vertex_file.begin(), vertex_file.end()};

    // Compile Shader
    if (is_vertex_shader) {
        this->id = glCreateShader(GL_VERTEX_SHADER);
        LOG_DEBUG(logger, "Compiling vertex shader:  {}", shader_path);
    } else {
        this->id = glCreateShader(GL_FRAGMENT_SHADER);
        LOG_DEBUG(logger, "Compiling fragment shader:  {}", shader_path);
    }

    char const* source_pointer = shader_code.c_str();
    glShaderSource(this->id, 1, &source_pointer, nullptr);
    glCompileShader(this->id);

    // Check Shader
    glGetShaderiv(this->id, GL_COMPILE_STATUS, &result);
    glGetShaderiv(this->id, GL_INFO_LOG_LENGTH, &info_log_length);
    if (info_log_length > 0) {
        std::vector<char> shader_error_message(info_log_length + 1);
        glGetShaderInfoLog(this->id, info_log_length, nullptr, &shader_error_message[0]);
        LOG_ERROR(logger, "{}", &shader_error_message[0]);
    }

    this->created = true;
}

GLShader::~GLShader() {
    if (created) {
        glDeleteShader(this->id);
    }
}

}  // namespace luhsoccer::luhviz