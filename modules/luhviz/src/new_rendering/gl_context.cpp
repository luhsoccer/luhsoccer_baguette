#include "include/gl_context.hpp"
#include <msdfgl.h>

#include <cmrc/cmrc.hpp>

CMRC_DECLARE(luhviz);

namespace luhsoccer::luhviz {

void glDebugOutput(GLenum source, GLenum type, unsigned int id, GLenum severity, GLsizei length, const char* message,
                   const void* userParam) {
    // ignore non-significant error/warning codes
    if (id == 131169 || id == 131185 || id == 131218 || id == 131204) return;

    std::cout << "---------------" << std::endl;
    std::cout << "Debug message (" << id << "): " << message << std::endl;

    switch (source) {
        case GL_DEBUG_SOURCE_API:
            std::cout << "Source: API";
            break;
        case GL_DEBUG_SOURCE_WINDOW_SYSTEM:
            std::cout << "Source: Window System";
            break;
        case GL_DEBUG_SOURCE_SHADER_COMPILER:
            std::cout << "Source: Shader Compiler";
            break;
        case GL_DEBUG_SOURCE_THIRD_PARTY:
            std::cout << "Source: Third Party";
            break;
        case GL_DEBUG_SOURCE_APPLICATION:
            std::cout << "Source: Application";
            break;
        case GL_DEBUG_SOURCE_OTHER:
            std::cout << "Source: Other";
            break;
    }
    std::cout << std::endl;

    switch (type) {
        case GL_DEBUG_TYPE_ERROR:
            std::cout << "Type: Error";
            break;
        case GL_DEBUG_TYPE_DEPRECATED_BEHAVIOR:
            std::cout << "Type: Deprecated Behaviour";
            break;
        case GL_DEBUG_TYPE_UNDEFINED_BEHAVIOR:
            std::cout << "Type: Undefined Behaviour";
            break;
        case GL_DEBUG_TYPE_PORTABILITY:
            std::cout << "Type: Portability";
            break;
        case GL_DEBUG_TYPE_PERFORMANCE:
            std::cout << "Type: Performance";
            break;
        case GL_DEBUG_TYPE_MARKER:
            std::cout << "Type: Marker";
            break;
        case GL_DEBUG_TYPE_PUSH_GROUP:
            std::cout << "Type: Push Group";
            break;
        case GL_DEBUG_TYPE_POP_GROUP:
            std::cout << "Type: Pop Group";
            break;
        case GL_DEBUG_TYPE_OTHER:
            std::cout << "Type: Other";
            break;
    }
    std::cout << std::endl;

    switch (severity) {
        case GL_DEBUG_SEVERITY_HIGH:
            std::cout << "Severity: high";
            break;
        case GL_DEBUG_SEVERITY_MEDIUM:
            std::cout << "Severity: medium";
            break;
        case GL_DEBUG_SEVERITY_LOW:
            std::cout << "Severity: low";
            break;
        case GL_DEBUG_SEVERITY_NOTIFICATION:
            std::cout << "Severity: notification";
            break;
    }
    std::cout << std::endl;
    std::cout << std::endl;
}

static msdfgl_context_t msdfgl_context;
static msdfgl_font_t msdfgl_font;

void GLContext::create() {
    msdfgl_context = msdfgl_create_context("330 core\n");
    // load text font
    auto fs = cmrc::luhviz::get_filesystem();
    auto file = fs.open("res/fonts/DroidSans.ttf");
    std::string data{file.begin(), file.end()};

    msdfgl_font = msdfgl_load_font_mem(msdfgl_context, &data[0], data.size(), 2.0, 3.0,
                                       nullptr); /* range, scale, atlas (NULL creates a new one) */

    // Loads characters 0-128 onto the textures. This is where all the GPU cycles went
    msdfgl_generate_ascii(msdfgl_font);

    // Enable auto-generating undefined glyphs as they are encountered for the first time
    msdfgl_set_missing_glyph_callback(msdfgl_context, msdfgl_generate_glyph, nullptr);
    created = true;
}

GLContext::~GLContext() {
    if (created) {
        msdfgl_destroy_font(msdfgl_font);
        msdfgl_destroy_context(msdfgl_context);
    }
}

void GLContext::displayText(float x, float y, float size, int32_t color, const std::string& text,
                            float* projection_matrix) {
    msdfgl_printf(x, y, msdfgl_font, size, color, projection_matrix, MSDFGL_KERNING, text.c_str(), MSDFGL_VERSION);
}

void GLContext::clear() { glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT); }

void GLContext::clearColor(GLclampf red, GLclampf green, GLclampf blue, GLclampf alpha) {
    glClearColor(red, green, blue, alpha);
}

void GLContext::useProgram(GLShaderProgram& program) { glUseProgram(program.getId()); }

void GLContext::bindTexture(GLTexture& texture, int unit) {
    glActiveTexture(GL_TEXTURE0 + unit);
    glBindTexture(GL_TEXTURE_2D, texture.getId());
}

void GLContext::drawElements(GLVertexArray& vao, GLsizei num_indices) {
    glBindVertexArray(vao.getId());
    glDrawElements(GL_TRIANGLES, num_indices, GL_UNSIGNED_SHORT, nullptr);
    glBindVertexArray(0);
}

void GLContext::drawArrays(GLVertexArray& vao, GLsizei size) {
    glBindVertexArray(vao.getId());
    glDrawArrays(GL_TRIANGLES, 0, size);
    glBindVertexArray(0);
}

void GLContext::bindFrameBuffer() {}

void GLContext::enable(GLenum which) { glEnable(which); }

void GLContext::disable(GLenum which) { glDisable(which); }

void GLContext::depthFunc(GLenum func) { glDepthFunc(func); }

void GLContext::blendFunc(GLenum sfactor, GLenum dfactor) { glBlendFunc(sfactor, dfactor); }

void GLContext::enableDebugging(bool enable) {
    if (enable) {
        glEnable(GL_DEBUG_OUTPUT);
        glEnable(GL_DEBUG_OUTPUT_SYNCHRONOUS);
        glDebugMessageCallback(glDebugOutput, nullptr);
        glDebugMessageControl(GL_DONT_CARE, GL_DONT_CARE, GL_DONT_CARE, 0, nullptr, GL_TRUE);
    }
}

}  // namespace luhsoccer::luhviz