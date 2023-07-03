#pragma once

#include "new_rendering/include/gl_shader_program.hpp"
#define GLM_ENABLE_EXPERIMENTAL
#include "glm/glm.hpp"
#include "glm/gtx/hash.hpp"
#include "logger/logger.hpp"
#include <unordered_map>
#include "common/include/utils.hpp"

namespace luhsoccer::luhviz {

struct Vertex {
    glm::vec3 pos;
    glm::vec4 color;
    glm::vec2 tex_coord;
    glm::vec3 normal;

    bool operator==(const Vertex& other) const {
        return pos == other.pos && color == other.color && tex_coord == other.tex_coord && normal == other.normal;
    }
};

class Model {
   public:
    Model(const std::string& model_path, GLShaderProgram& program);

    std::vector<Vertex> getVertices() { return this->vertices; }
    std::vector<uint16_t> getIndices() { return this->indices; }
    GLShaderProgram& getProgram() { return this->program; }

   private:
    logger::Logger logger{"luhviz/model"};
    std::vector<Vertex> vertices;
    std::vector<uint16_t> indices;
    GLShaderProgram& program;

    void loadModel(const std::string& model_path);
};

}  // namespace luhsoccer::luhviz

namespace std {
template <>
struct hash<luhsoccer::luhviz::Vertex> {
    size_t operator()(luhsoccer::luhviz::Vertex const& vertex) const {
        return ((hash<glm::vec3>()(vertex.pos) ^ (hash<glm::vec3>()(vertex.color) << 1)) >> 1) ^
               (hash<glm::vec2>()(vertex.tex_coord) << 1);
    }
};
}  // namespace std