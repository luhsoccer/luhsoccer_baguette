#include "include/model.hpp"

#define TINYOBJLOADER_IMPLEMENTATION
#include "tiny_obj_loader.h"
#include <cmrc/cmrc.hpp>

CMRC_DECLARE(luhviz);

namespace luhsoccer::luhviz {
Model::Model(const std::string& model_path, GLShaderProgram& program) : program(program) { loadModel(model_path); }

void Model::loadModel(const std::string& model_path) {
    auto fs = cmrc::luhviz::get_filesystem();
    auto file = fs.open(model_path);
    std::string data{file.begin(), file.end()};

    tinyobj::ObjReader reader{};
    tinyobj::ObjReaderConfig reader_config;
    reader_config.triangulate = true;  // we can assume that every face consists of 3 vertices
    reader_config.vertex_color = false;

    if (!reader.ParseFromString(data, "", reader_config)) {
        if (!reader.Error().empty()) {
            LOG_ERROR(logger, "TinyObjReader ({}): {}", model_path, reader.Error());
        }
    }

    if (!reader.Warning().empty()) {
        LOG_WARNING(logger, "TinyObjReader ({}): {}", model_path, reader.Warning());
    }

    // get model data
    tinyobj::attrib_t attrib = reader.GetAttrib();
    std::vector<tinyobj::shape_t> shapes = reader.GetShapes();

    // index model data
    std::unordered_map<Vertex, uint16_t> unique_vertices{};
    for (const auto& shape : shapes) {
        for (const auto& index : shape.mesh.indices) {
            Vertex vertex{};
            vertex.pos = {attrib.vertices[3 * index.vertex_index + 0], attrib.vertices[3 * index.vertex_index + 1],
                          attrib.vertices[3 * index.vertex_index + 2]};

            vertex.tex_coord = {attrib.texcoords[2 * index.texcoord_index + 0],
                                1.0f - attrib.texcoords[2 * index.texcoord_index + 1]};

            vertex.normal = {attrib.normals[3 * index.normal_index + 0], attrib.normals[3 * index.normal_index + 1],
                             attrib.normals[3 * index.normal_index + 2]};

            vertex.color = {1.0f, 1.0f, 1.0f, 1.0f};

            if (unique_vertices.count(vertex) == 0) {
                unique_vertices.try_emplace(vertex, vertices.size());
                vertices.push_back(vertex);
            }

            indices.push_back(unique_vertices[vertex]);
        }
    }
}
}  // namespace luhsoccer::luhviz