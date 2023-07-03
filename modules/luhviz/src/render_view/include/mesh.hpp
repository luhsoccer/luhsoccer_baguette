#pragma once

#include <utility>

#include "new_rendering/include/gl_context.hpp"
#include "new_rendering/include/gl_shader_program.hpp"
#include "new_rendering/include/gl_texture.hpp"
#include "new_rendering/include/gl_uniform.hpp"
#include "new_rendering/include/gl_vertex_array.hpp"
#include "new_rendering/include/gl_buffer.hpp"
#include "render_view/include/model.hpp"
#include "vector"
#include "glm/glm.hpp"

namespace luhsoccer::luhviz {

class TextMesh {
   public:
    TextMesh() = delete;
    TextMesh(std::unique_ptr<GLContext>& context) : context(context) {}

    void setMVP(glm::mat4 mvp) { this->mvp = mvp; }
    void setColor(glm::vec4 color) { this->color = color; }
    void setText(std::string text) { this->text = std::move(text); }
    [[nodiscard]] glm::mat4 getMVP() const { return this->mvp; }
    [[nodiscard]] glm::vec4 getColor() const { return this->color; }
    [[nodiscard]] std::string getText() const { return this->text; }

    void render() { this->context->displayText(0, 0, 0.1f, color2Hex(this->color), this->text, &this->mvp[0][0]); }

   private:
    constexpr static float COL_MAX = 255.0f;

    std::unique_ptr<GLContext>& context;
    glm::mat4 mvp{};
    glm::vec4 color{};
    std::string text{};

    [[nodiscard]] int color2Hex(glm::vec4 color) const {
        return (((int)(color.r * COL_MAX)) << 24) | (((int)(color.g * COL_MAX)) << 16) |
               (((int)(color.b * COL_MAX)) << 8) | ((int)(color.a * COL_MAX));
    }
};

/**
 * @brief this class represents a 3d mesh (loaded .obj files)
 *
 */
class Mesh {
   public:
    ~Mesh() = default;
    Mesh(const Mesh&) = default;
    Mesh(Mesh&&) = delete;
    Mesh& operator=(const Mesh&) = delete;
    Mesh& operator=(Mesh&&) = delete;
    Mesh(std::unique_ptr<GLContext>& context, GLUniformMat4vf mvp_id, GLUniformMat4vf mv_id, GLUniformMat4vf m_id,
         GLUniformMat4vf v_id, GLUniform3f light_pos_id, GLUniform1i type_id)
        : context(context),
          type_id(type_id),
          mvp_id(mvp_id),
          mv_id(mv_id),
          m_id(m_id),
          v_id(v_id),
          light_pos_id(light_pos_id) {}

    // create mesh from loaded model with color
    Mesh(Model& model, std::unique_ptr<GLContext>& context, GLUniformMat4vf mvp_id, GLUniformMat4vf mv_id,
         GLUniformMat4vf m_id, GLUniformMat4vf v_id, GLUniform3f light_pos_id, GLUniform1i type_id, glm::vec4 color,
         GLUniform4f color_id)
        : Mesh(context, mvp_id, mv_id, m_id, v_id, light_pos_id, type_id) {
        this->color = color;
        this->opaque = this->color.a >= 1.0;
        this->color_id = color_id;
        this->type = 2;

        createMeshFromModel(model);
    }

    // create mesh from loaded model with texture
    Mesh(Model& model, std::unique_ptr<GLContext>& context, GLUniformMat4vf& mvp_id, GLUniformMat4vf mv_id,
         GLUniformMat4vf m_id, GLUniformMat4vf v_id, GLUniform3f light_pos_id, GLUniform1i type_id,
         const std::shared_ptr<GLTexture>& texture, GLUniform1i texture_id)
        : Mesh(context, mvp_id, mv_id, m_id, v_id, light_pos_id, type_id) {
        //
        this->texture = texture;
        this->texture_id = texture_id;
        this->type = 0;

        createMeshFromModel(model);
    }

    // create mesh from vertex data (generated from 2d markers)
    Mesh(std::unique_ptr<GLContext>& context, GLUniformMat4vf mvp_id, GLUniformMat4vf mv_id, GLUniformMat4vf m_id,
         GLUniformMat4vf v_id, GLUniform3f light_pos_id, GLUniform1i type_id, const std::vector<glm::vec3>& vertices,
         std::vector<glm::vec4> colors, glm::vec4 color, GLUniform4f color_id, GLUniform1i multicolor_id)
        : Mesh(context, mvp_id, mv_id, m_id, v_id, light_pos_id, type_id) {
        this->colors = std::move(colors);
        this->color = color;
        this->color_id = color_id;
        this->multicolor_id = multicolor_id;
        this->type = 1;

        //
        // check if it has transparent parts
        auto it = std::find_if(this->colors.begin(), this->colors.end(), [](glm::vec4 c) { return c.a != 1.0; });
        if (it != this->colors.end()) {
            this->opaque = false;
        }

        // create vao with vertices
        this->vao.bind();
        this->data_buffer.bufferArrayDataVert(vertices);
        this->vao.bindAttribute(VERTEX_ATTR_IND, 3, (void*)nullptr, sizeof(glm::vec3));
        if (!this->colors.empty()) {
            this->color_buffer.bufferArrayDataColor(this->colors);
            this->vao.bindAttribute(COLOR_ATTR_IND, 4, (void*)nullptr, sizeof(glm::vec4));
        }
        this->vao.unbind();

        this->vertex_count = static_cast<GLsizei>(vertices.size());
        this->is_2d = true;
    }

    void createMeshFromModel(Model& model) {
        // create vao with vertices, normals and uvs
        this->vao.bind();
        this->data_buffer.bufferArrayData(model.getVertices());
        // the data in the data buffer relies on the Vertex struct. Because structs memory layout is sequential we have
        // always vertex,normal,uv and repeat so we can define this as offset in the data
        int struct_size = sizeof(Vertex);
        this->vao.bindAttribute(VERTEX_ATTR_IND, 3, (void*)nullptr, struct_size);
        this->vao.bindAttribute(NORMAL_ATTR_IND, 3, (void*)offsetof(Vertex, normal), struct_size);
        this->vao.bindAttribute(TEXCOORD_ATTR_IND, 2, (void*)offsetof(Vertex, tex_coord), struct_size);
        // indices
        this->element_buffer.bufferElementData(model.getIndices());
        this->vao.bindElements(this->element_buffer);
        this->vao.unbind();

        this->vertex_count = static_cast<GLsizei>(model.getVertices().size());
        this->index_count = static_cast<GLsizei>(model.getIndices().size());
    }

    void render() {
        // set color/texture attributes
        if (texture != nullptr) {
            this->context->bindTexture(*this->texture, 0);
            this->texture_id.set(0);
        } else {
            this->color_id.set(this->color.r, this->color.g, this->color.b, this->color.a);
            if (is_2d) {
                this->multicolor_id.set(this->colors.empty() ? 0 : 1);
            }
        }

        // set transformation matrix
        this->mvp_id.set(&this->mvp[0][0]);
        this->mv_id.set(&this->mv[0][0]);
        this->m_id.set(&this->m[0][0]);
        this->v_id.set(&this->v[0][0]);
        this->light_pos_id.set(this->light_pos.x, this->light_pos.y, this->light_pos.z);
        this->type_id.set(this->type);

        // draw mesh
        if (this->index_count == 0) {
            this->context->drawArrays(this->vao, this->vertex_count);
        } else {
            this->context->drawElements(this->vao, this->index_count);
        }
    }

    [[nodiscard]] bool isOpaque() const { return this->opaque; }

    [[nodiscard]] bool is2d() const { return this->is_2d; }

    [[nodiscard]] glm::vec4 getColor() const { return this->color; }

    [[nodiscard]] glm::mat4 getMVP() const { return this->mvp; }

    void setMVP(glm::mat4 mvp) { this->mvp = mvp; }

    void setOrigin(glm::vec3 origin) { this->origin = origin; }

    [[nodiscard]] glm::vec3 getOrigin() const { return origin; }

    void setLightPos(glm::vec3 pos) { this->light_pos = pos; }

    void setM(glm::mat4 m) { this->m = m; }

    void setV(glm::mat4 v) { this->v = v; }

    void setMV(glm::mat4 mv) { this->mv = mv; }

    void setVertices(const std::vector<glm::vec3>& vertices) {
        glBindBuffer(GL_ARRAY_BUFFER, this->data_buffer.getId());
        glBufferData(GL_ARRAY_BUFFER, static_cast<GLsizeiptr>(vertices.size() * sizeof(glm::vec3)), nullptr,
                     GL_DYNAMIC_DRAW);
        this->data_buffer.bufferArrayDataVert(vertices);
        // update vertices size
        this->vertex_count = static_cast<GLsizei>(vertices.size());
    }

    void setColors(const std::vector<glm::vec4>& colors) { this->color_buffer.bufferArrayDataColor(colors); }

    void setColor(const glm::vec4& color) { this->color = color; }

   private:
    luhsoccer::logger::Logger logger{"luhviz/mesh"};

    constexpr static int VERTEX_ATTR_IND = 0;
    constexpr static int NORMAL_ATTR_IND = 1;
    constexpr static int TEXCOORD_ATTR_IND = 2;
    constexpr static int COLOR_ATTR_IND = 3;

    bool opaque{true};
    bool is_2d{false};
    glm::vec3 origin{};

    // context and buffers
    std::unique_ptr<GLContext>& context;
    GLVertexArray vao{};
    GLBuffer data_buffer{};
    GLBuffer element_buffer{};
    GLBuffer color_buffer{};
    GLsizei vertex_count{0};
    GLsizei index_count{0};
    // uniforms
    GLUniform4f color_id;
    glm::vec4 color{};
    GLUniform1i multicolor_id;
    GLUniform1i type_id;
    int type{2};  // default type is colored mesh
    std::vector<glm::vec4> colors{};
    GLUniformMat4vf mvp_id;
    glm::mat4 mvp{};
    GLUniformMat4vf mv_id;
    glm::mat4 mv{};
    GLUniformMat4vf m_id;
    glm::mat4 m{};
    GLUniformMat4vf v_id;
    glm::mat4 v{};
    GLUniform3f light_pos_id;
    glm::vec3 light_pos{};
    GLUniform1i texture_id;
    std::shared_ptr<GLTexture> texture{nullptr};
};
}  // namespace luhsoccer::luhviz
