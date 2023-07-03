#include "include/gl_texture.hpp"

#define STB_IMAGE_IMPLEMENTATION
#include <stb_image.h>

#include <cmrc/cmrc.hpp>

CMRC_DECLARE(luhviz);

namespace luhsoccer::luhviz {

GLTexture::GLTexture(const std::string& image_path, bool flip_vertically) { create(image_path, flip_vertically); }

bool GLTexture::create(const std::string& image_path, bool flip_vertically, bool has_alpha) {
    // if (this->created) {
    //     // if already created, release memory first
    //     stbi_image_free(this->img_data);
    //     glDeleteTextures(1, &this->id);
    // }

    // Create one OpenGL texture
    glGenTextures(1, &this->id);
    // "Bind" the newly created texture
    glBindTexture(GL_TEXTURE_2D, this->id);

    // sometimes the images needs to be flipped
    stbi_set_flip_vertically_on_load(flip_vertically);

    // load image data
    auto fs = cmrc::luhviz::get_filesystem();
    if (!fs.exists(image_path)) return false;
    auto file = fs.open(image_path);
    std::string data{file.begin(), file.end()};
    if (has_alpha) {
        this->img_data =
            stbi_load_from_memory(reinterpret_cast<const unsigned char*>(data.c_str()), static_cast<int>(file.size()),
                                  &this->width, &this->height, &this->num_channels, STBI_rgb_alpha);
        // Give the image to OpenGL
        glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, this->width, this->height, 0, GL_RGBA, GL_UNSIGNED_BYTE,
                     this->img_data);
    } else {
        this->img_data =
            stbi_load_from_memory(reinterpret_cast<const unsigned char*>(data.c_str()), static_cast<int>(file.size()),
                                  &this->width, &this->height, &this->num_channels, STBI_rgb);
        // Give the image to OpenGL
        glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, this->width, this->height, 0, GL_RGB, GL_UNSIGNED_BYTE, this->img_data);
    }

    // simple filtering
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);

    this->created = true;
    return true;
}

GLTexture::~GLTexture() {
    if (this->created) {
        stbi_image_free(this->img_data);
        glDeleteTextures(1, &this->id);
    }
}

GLuint GLTexture::getId() { return this->id; }

ImTextureID GLTexture::getImguiId() { return (ImTextureID)this->getId(); }

}  // namespace luhsoccer::luhviz