#include "include/gl_uniform.hpp"

namespace luhsoccer::luhviz {
// Uniform1f
GLUniform1F::GLUniform1F(GLuint program_id, const std::string& name)
    : id(glGetUniformLocation(program_id, name.c_str())) {}
GLint GLUniform1F::getId() const { return this->id; }
void GLUniform1F::set(const GLfloat& value) { glUniform1f(this->id, value); }

// Uniform2f
GLUniform2F::GLUniform2F(GLuint program_id, const std::string& name)
    : id(glGetUniformLocation(program_id, name.c_str())) {}
GLint GLUniform2F::getId() const { return this->id; }
void GLUniform2F::set(const GLfloat& value1, const GLfloat& value2) { glUniform2f(this->id, value1, value2); }

// Uniform3f
GLUniform3f::GLUniform3f(GLuint program_id, const std::string& name)
    : id(glGetUniformLocation(program_id, name.c_str())) {}
GLint GLUniform3f::getId() const { return this->id; }
void GLUniform3f::set(const GLfloat& value1, const GLfloat& value2, const GLfloat& value3) {
    glUniform3f(this->id, value1, value2, value3);
}

// Uniform4f
GLUniform4f::GLUniform4f(GLuint program_id, const std::string& name)
    : id(glGetUniformLocation(program_id, name.c_str())) {}
GLint GLUniform4f::getId() const { return this->id; }
void GLUniform4f::set(const GLfloat& value1, const GLfloat& value2, const GLfloat& value3, const GLfloat& value4) {
    glUniform4f(this->id, value1, value2, value3, value4);
}

// Uniform1i
GLUniform1i::GLUniform1i(GLuint program_id, const std::string& name)
    : id(glGetUniformLocation(program_id, name.c_str())) {}
GLint GLUniform1i::getId() const { return this->id; }
void GLUniform1i::set(const GLint& value) { glUniform1i(this->id, value); }

// UniformMat4fv
GLUniformMat4vf::GLUniformMat4vf(GLuint program_id, const std::string& name)
    : id(glGetUniformLocation(program_id, name.c_str())) {}
GLint GLUniformMat4vf::getId() const { return this->id; }
void GLUniformMat4vf::set(const GLfloat* first_value) { glUniformMatrix4fv(this->id, 1, GL_FALSE, first_value); }

}  // namespace luhsoccer::luhviz