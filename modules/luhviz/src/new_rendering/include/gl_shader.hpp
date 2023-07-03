#pragma once

#include <glad/glad.h>
#include "logger/logger.hpp"

namespace luhsoccer::luhviz {

class GLShader {
   public:
    GLShader(const GLShader &) = default;
    GLShader(GLShader &&) = delete;
    GLShader &operator=(const GLShader &) = default;
    GLShader &operator=(GLShader &&) = default;

    GLShader() = default;

    ~GLShader();

    void create(const std::string &shader_path, bool is_vertex_shader);

    [[nodiscard]] GLuint getId() const { return this->id; }

   private:
    logger::Logger logger{"Luhviz/GLShader"};

    GLuint id{};
    bool created{false};
};

}  // namespace luhsoccer::luhviz