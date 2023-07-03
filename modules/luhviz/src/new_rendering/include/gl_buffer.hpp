#pragma once

#include <glad/glad.h>
#include "logger/logger.hpp"
#include "glm/glm.hpp"
#include "render_view/include/model.hpp"

namespace luhsoccer::luhviz {

class GLBuffer {
   public:
    GLBuffer(const GLBuffer&) = default;
    GLBuffer(GLBuffer&&) = delete;
    GLBuffer& operator=(const GLBuffer&) = default;
    GLBuffer& operator=(GLBuffer&&) = delete;

    /**
     * @brief Construct a new GLVertexBuffer
     *
     * @param vertex_shader_path
     * @param fragment_shader_path
     */
    GLBuffer();

    /**
     * @brief Destroy the GLVertexBuffer and free data
     *
     */
    ~GLBuffer();

    void bufferArrayData(const std::vector<Vertex>& buffer_data);
    void bufferArrayData(const std::vector<GLfloat>& buffer_data);
    void bufferArrayDataVert(const std::vector<glm::vec3>& buffer_data);
    void bufferArrayDataUV(const std::vector<glm::vec2>& buffer_data);
    void bufferArrayDataColor(const std::vector<glm::vec4>& buffer_data);
    void bufferElementData(const std::vector<uint16_t>& buffer_data);

    [[nodiscard]] GLuint getId() const { return this->id; }

   private:
    logger::Logger logger{"Luhviz/GLVertexBuffer"};

    GLuint id;
};

}  // namespace luhsoccer::luhviz