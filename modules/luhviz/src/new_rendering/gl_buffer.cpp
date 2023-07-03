#include "include/gl_buffer.hpp"

namespace luhsoccer::luhviz {
GLBuffer::GLBuffer() { glGenBuffers(1, &this->id); }

void GLBuffer::bufferArrayData(const std::vector<Vertex>& buffer_data) {
    if (buffer_data.empty()) return;
    glBindBuffer(GL_ARRAY_BUFFER, this->id);
    glBufferData(GL_ARRAY_BUFFER, static_cast<GLsizeiptr>(buffer_data.size() * sizeof(Vertex)), &buffer_data[0],
                 GL_STATIC_DRAW);
}

void GLBuffer::bufferArrayData(const std::vector<GLfloat>& buffer_data) {
    if (buffer_data.empty()) return;
    glBindBuffer(GL_ARRAY_BUFFER, this->id);
    glBufferData(GL_ARRAY_BUFFER, static_cast<GLsizeiptr>(buffer_data.size() * sizeof(GLfloat)), &buffer_data[0],
                 GL_STATIC_DRAW);
}

void GLBuffer::bufferArrayDataVert(const std::vector<glm::vec3>& buffer_data) {
    if (buffer_data.empty()) return;
    glBindBuffer(GL_ARRAY_BUFFER, this->id);
    glBufferData(GL_ARRAY_BUFFER, static_cast<GLsizeiptr>(buffer_data.size() * sizeof(glm::vec3)), &buffer_data[0],
                 GL_DYNAMIC_DRAW);
}

void GLBuffer::bufferArrayDataUV(const std::vector<glm::vec2>& buffer_data) {
    if (buffer_data.empty()) return;
    glBindBuffer(GL_ARRAY_BUFFER, this->id);
    glBufferData(GL_ARRAY_BUFFER, static_cast<GLsizeiptr>(buffer_data.size() * sizeof(glm::vec2)), &buffer_data[0],
                 GL_STATIC_DRAW);
}

void GLBuffer::bufferArrayDataColor(const std::vector<glm::vec4>& buffer_data) {
    if (buffer_data.empty()) return;
    glBindBuffer(GL_ARRAY_BUFFER, this->id);
    glBufferData(GL_ARRAY_BUFFER, static_cast<GLsizeiptr>(buffer_data.size() * sizeof(glm::vec4)), &buffer_data[0],
                 GL_STATIC_DRAW);
}

void GLBuffer::bufferElementData(const std::vector<uint16_t>& buffer_data) {
    if (buffer_data.empty()) return;
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, this->id);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, static_cast<GLsizeiptr>(buffer_data.size() * sizeof(uint16_t)),
                 &buffer_data[0], GL_STATIC_DRAW);
}

GLBuffer::~GLBuffer() {  //
    glDeleteBuffers(1, &this->id);
}
}  // namespace luhsoccer::luhviz