#pragma once

#include <glad/glad.h>
#include "logger/logger.hpp"
#include "new_rendering/include/gl_buffer.hpp"

namespace luhsoccer::luhviz {

class GLVertexArray {
   public:
    GLVertexArray(const GLVertexArray&) = default;
    GLVertexArray(GLVertexArray&&) = delete;
    GLVertexArray& operator=(const GLVertexArray&) = default;
    GLVertexArray& operator=(GLVertexArray&&) = delete;

    /**
     * @brief Construct a new GLVertexArray
     *
     * @param vertex_shader_path
     * @param fragment_shader_path
     */
    GLVertexArray();

    /**
     * @brief Destroy the GLVertexArray and free data
     *
     */
    ~GLVertexArray();

    void bind();

    void unbind();

    /**
     * @brief binds attributes such as vertex- or colorbuffer to this vao
     *
     * @param vertex_buffer the buffer to bind
     * @param index the index of the attribute
     * @param values_count the number of components (values), must be 1,2,3 or 4
     * eg. position(x,y,z) = 3, color(r,g,b,a) = 4
     */
    void bindAttribute(int index, size_t values_count, void* offset, int struct_size);

    /**
     * @brief binds elements buffer such as indexed vertices
     *
     * @param elements the indexed elements buffer
     */
    void bindElements(const GLBuffer& elements);

    [[nodiscard]] GLuint getId() const;

   private:
    logger::Logger logger{"Luhviz/GLVertexArray"};

    GLuint id{};
};

}  // namespace luhsoccer::luhviz