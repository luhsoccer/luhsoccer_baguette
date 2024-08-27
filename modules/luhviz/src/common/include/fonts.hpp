#pragma once

#include "imgui.h"
#include "logger/logger.hpp"

#include "cmrc/cmrc.hpp"

CMRC_DECLARE(luhviz);

namespace luhsoccer::luhviz {

class Fonts {
   public:
    constexpr static int FONT_STANDARD = 0;
    constexpr static int FONT_LARGE = 1;
    constexpr static int FONT_SMALL = 2;
    constexpr static int FONT_DEBUGGER_MAIN = 3;
    constexpr static int FONT_DEBUGGER_MONO = 4;
    constexpr static int FONT_BAGUETTE_MODE = 5;
    constexpr static int FONT_XL = 6;
    constexpr static int FONT_XXL = 7;
    constexpr static int FONT_MEDIUM = 8;

    /**
     * @brief loads all fonts at startup
     *
     */
    void loadFonts() {
        ImGuiIO& io = ImGui::GetIO();
        // load standard font
        const float size_std = 16.0f;
        font_std = loadFont(size_std, io);
        // load small font
        const float size_sm = 10.0f;
        font_sm = loadFont(size_sm, io);
        // load large font
        const float size_lg = 20.0f;
        font_lg = loadFont(size_lg, io);

        const float size_xl = 30.0f;
        font_xl = loadFont(size_xl, io);

        const float size_xxl = 40.0f;
        font_xxl = loadFont(size_xxl, io);

        const float size_medium = 13.0f;
        font_medium = loadFont(size_medium, io);

        // =============== Load fonts with special characters ===============
        ImFontGlyphRangesBuilder builder;
        ImVector<ImWchar> ranges;
        builder.AddRanges(io.Fonts->GetGlyphRangesDefault());
        builder.AddRanges(io.Fonts->GetGlyphRangesCyrillic());
        builder.AddText("░▓▒█");  // These characters are used for baguette mode, do not change please
        builder.AddText("αβγδεζηθικλμνξοπρστυφχψωςΑΒΓΔΕΖΗΘΙΚΛΜΝΗΟΠΡΣΤΥΦΧΨΩάέήίϊΐόύϋΰώΆΈΉΊΪΌΎΫΫ́Ώ");
        builder.AddText("×");
        builder.AddText("äöüß");
        builder.BuildRanges(&ranges);

        // load main debugger font
        const float size_debugger_main_font = 16.0f;
        font_debugger_main = loadSpecialFont(size_debugger_main_font, io, "Roboto-Regular.ttf", ranges);
        // io.Fonts->Build();
        //   load mono debugger font
        const float size_debugger_mono_font = 16.0f;
        font_debugger_mono = loadSpecialFont(size_debugger_mono_font, io, "RobotoMono-Regular.ttf", ranges);
        // io.Fonts->Build();
        //   load baguette mode font
        const float size_baguette_mode_font = 3.0f;
        font_baguette_mode = loadSpecialFont(size_baguette_mode_font, io, "Arimo-Regular.ttf", ranges);
        // io.Fonts->Build();
    }

    /**
     * @brief Get the Font object
     *
     * @param font_code
     * @return ImFont
     */
    ImFont* getFont(int font_code) {
        switch (font_code) {
            case FONT_STANDARD:
                return font_std;
            case FONT_SMALL:
                return font_sm;
            case FONT_LARGE:
                return font_lg;
            case FONT_XL:
                return font_xl;
            case FONT_XXL:
                return font_xxl;
            case FONT_MEDIUM:
                return font_medium;
            case FONT_DEBUGGER_MAIN:
                return font_debugger_main;
            case FONT_DEBUGGER_MONO:
                return font_debugger_mono;
            case FONT_BAGUETTE_MODE:
                return font_baguette_mode;

                break;
        }
        return {};
    }

   private:
    ImFont* font_std;
    ImFont* font_sm;
    ImFont* font_lg;
    ImFont* font_debugger_main;
    ImFont* font_debugger_mono;
    ImFont* font_baguette_mode;
    ImFont* font_xl;
    ImFont* font_xxl;
    ImFont* font_medium;

    ImFont* loadSpecialFont(float pixel_size, const ImGuiIO& io, const std::string& font_file_name,
                            ImVector<ImWchar>& ranges) {
        ImFontConfig conf;
        conf.SizePixels = pixel_size;
        conf.FontDataOwnedByAtlas = false;

        auto fs = cmrc::luhviz::get_filesystem();
        std::string font_file_path = "res/fonts/";
        font_file_path.append(font_file_name);

        auto ttf_file = fs.open(font_file_path.c_str());
        std::string font{ttf_file.begin(), ttf_file.end()};

        ImFont* loaded_font = io.Fonts->AddFontFromMemoryTTF((void*)ttf_file.begin(), static_cast<int>(ttf_file.size()),
                                                             pixel_size, &conf, ranges.Data);

        io.Fonts->Build();
        return loaded_font;
    }

    ImFont* loadFont(float pixel_size, const ImGuiIO& io) {
        ImFontConfig conf;
        conf.SizePixels = pixel_size;
        conf.FontDataOwnedByAtlas = false;

        auto fs = cmrc::luhviz::get_filesystem();
        auto ttf_file = fs.open("res/fonts/DroidSans.ttf");
        std::string font{ttf_file.begin(), ttf_file.end()};

        return io.Fonts->AddFontFromMemoryTTF((void*)ttf_file.begin(), static_cast<int>(ttf_file.size()), pixel_size,
                                              &conf);
    }
};

}  // namespace luhsoccer::luhviz