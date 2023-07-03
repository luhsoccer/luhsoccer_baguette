#pragma once

#include <glad/glad.h>
#include "new_rendering/include/gl_shader_program.hpp"

namespace luhsoccer::luhviz {

class GLUniform1F {
   public:
    GLUniform1F() = default;
    GLUniform1F(GLuint program_id, const std::string& name);
    [[nodiscard]] GLint getId() const;
    void set(const GLfloat& value);

   private:
    GLint id{};
};

class GLUniform2F {
   public:
    GLUniform2F() = default;
    GLUniform2F(GLuint program_id, const std::string& name);
    [[nodiscard]] GLint getId() const;
    void set(const GLfloat& value1, const GLfloat& value2);

   private:
    GLint id{};
};

class GLUniform3f {
   public:
    GLUniform3f() = default;
    GLUniform3f(GLuint program_id, const std::string& name);
    [[nodiscard]] GLint getId() const;
    void set(const GLfloat& value1, const GLfloat& value2, const GLfloat& value3);

   private:
    GLint id{};
};

class GLUniform4f {
   public:
    GLUniform4f() = default;
    GLUniform4f(GLuint program_id, const std::string& name);
    [[nodiscard]] GLint getId() const;
    void set(const GLfloat& value1, const GLfloat& value2, const GLfloat& value3, const GLfloat& value4);

   private:
    GLint id{};
};

class GLUniform1i {
   public:
    GLUniform1i() = default;
    GLUniform1i(GLuint program_id, const std::string& name);
    [[nodiscard]] GLint getId() const;
    void set(const GLint& value);

   private:
    GLint id{};
};

class GLUniformMat4vf {
   public:
    GLUniformMat4vf() = default;
    GLUniformMat4vf(GLuint program_id, const std::string& name);
    [[nodiscard]] GLint getId() const;
    void set(const GLfloat* first_value);

   private:
    GLint id{};
};

}  // namespace luhsoccer::luhviz