#include "include/gl_vertex_array.hpp"

namespace luhsoccer::luhviz {
GLVertexArray::GLVertexArray() { glGenVertexArrays(1, &this->id); }

void GLVertexArray::bind() { glBindVertexArray(this->id); }
void GLVertexArray::unbind() { glBindVertexArray(0); }

void GLVertexArray::bindAttribute(int index, size_t values_count, void* offset, int struct_size) {
    glEnableVertexAttribArray(index);
    glVertexAttribPointer(index, static_cast<GLint>(values_count), GL_FLOAT, GL_FALSE, struct_size, offset);
}

void GLVertexArray::bindElements(const GLBuffer& elements) { glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, elements.getId()); }

GLuint GLVertexArray::getId() const { return this->id; }

GLVertexArray::~GLVertexArray() {
    //
    glDeleteVertexArrays(1, &this->id);
}
}  // namespace luhsoccer::luhviz