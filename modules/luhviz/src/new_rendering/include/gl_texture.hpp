#pragma once

#include <glad/glad.h>
#include <imgui.h>
#include "logger/logger.hpp"

namespace luhsoccer::luhviz {

/**
 * @brief This class is a wrapper representing an opengl texture
 */
class GLTexture {
   public:
    GLTexture(const GLTexture&) = default;
    GLTexture(GLTexture&&) = default;
    GLTexture& operator=(const GLTexture&) = default;
    GLTexture& operator=(GLTexture&&) = default;
    GLTexture() = default;

    GLTexture(const std::string& image_path, bool flip_vertically);

    /**
     * @brief Construct a new GLTexture
     *
     * @param image_path the path of the cmake_rc image source
     * @param flip_vertically true = flips the pixels vertically
     */
    bool create(const std::string& image_path, bool flip_vertically, bool has_alpha = false);

    /**
     * @brief Destroy the GLTexture and frees allocated data
     *
     */
    ~GLTexture();

    GLuint getId();

    /**
     * @brief returns if is already created
     *
     * @return true
     * @return false
     */
    bool isCreated() { return this->created; }

    ImTextureID getImguiId();

    [[nodiscard]] int getWidth() const { return this->width; }
    [[nodiscard]] int getHeight() const { return this->height; }
    unsigned char* getImageData() { return this->img_data; }

   private:
    logger::Logger logger{"Luhviz/GLTexture"};

    GLuint id{};
    unsigned char* img_data{};
    int num_channels = 0;
    int width = 0;
    int height = 0;

    bool created{false};
};

}  // namespace luhsoccer::luhviz