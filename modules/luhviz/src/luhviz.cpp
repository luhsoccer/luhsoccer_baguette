#include <glad/glad.h>
#include <vector>
#include "game_controller_window/include/game_controller_window.hpp"
#include "imgui.h"
#include "imgui_backend/imgui_impl_glfw.h"
#include "imgui_backend/imgui_impl_opengl3.h"
#include <GLFW/glfw3.h>
#include <iostream>
#include <fstream>
#include "info_display/include/info_display.hpp"
#include "marker_service/marker_service.hpp"
#include "marker_service/marker_test.hpp"
#include "plotter/include/plotter.hpp"
#include "scenario/scenario_executor.hpp"
#include "ssl_interface/ssl_interface.hpp"
#include "game_data_provider/game_data_provider.hpp"
#include "config/config_store.hpp"
#include "config_provider/config_store_main.hpp"
#include "simulation_interface/simulation_interface.hpp"
#include "local_planner/local_planner_module.hpp"
#include "skill_books/bod_skill_book.hpp"
#include <imgui_internal.h>
#include <csignal>
#include <filesystem>
#include "debugger/include/debugger.hpp"
#include "game_info/include/game_info.hpp"
#include "inspector/include/inspector.hpp"
#include "luhconfig/include/luhconfig.hpp"
#include "main_window/include/main_window.hpp"
#include "render_view/include/render_view.hpp"
#include "common/include/fonts.hpp"
#include "luhviz/luhviz.hpp"
#include "include/data_proxy.hpp"
#include "cmrc/cmrc.hpp"
#include "robot_info_display/include/robot_info_display.hpp"
#include "skill_tester/include/skill_tester.hpp"
#include "skill_wizard/include/skill_wizard.hpp"
#include "robot_controller/include/robot_controller.hpp"
#include "marker_service/marker_2d_impl.hpp"
#include "marker_service/marker_impl.hpp"
#include "marker_service/luhviz_impl.hpp"
#include "logger/gui_sink.hpp"

CMRC_DECLARE(luhviz);

namespace luhsoccer::luhviz {

const static logger::Logger LOGGER{"luhviz"};

static void glfwErrorCallback(int error, const char* description) {
    LOG_ERROR(LOGGER, "Glfw Error: {}: {}", error, description);
}

class LuhvizMain::LuhvizMainImpl {
   public:
    LuhvizMainImpl(marker::MarkerService& ms, ssl_interface::SSLInterface& ssl,
                   simulation_interface::SimulationInterface& sim, game_data_provider::GameDataProvider& gdp,
                   local_planner::LocalPlannerModule& local_planner, skills::BodSkillBook& skill_book,
                   robot_interface::RobotInterface& robot_interface, scenario::ScenarioExecutor& scenario_executor)
        : marker_service(ms),
          ssl(ssl),
          sim{sim},
          gdp(gdp),
          local_planner(local_planner),
          skill_book(skill_book),
          robot_interface(robot_interface),
          scenario_executor(scenario_executor){};

    void setup() {
        if (std::getenv("BAGUETTE_HEADLESS")) {
            // TODO: don't disable everything from luhviz
            // web api should be enabled
            LOG_INFO(LOGGER, "Starting in headless mode");
        } else {
            this->enable = true;
            init();
        }
    }

    void init() {
        // Setup window
        glfwSetErrorCallback(glfwErrorCallback);
        if (!glfwInit()) {
            LOG_ERROR(LOGGER, "could not init glfw");
            return;
        }
        constexpr bool DEBUGGING_OPENGL = false;
        glfwWindowHint(GLFW_OPENGL_DEBUG_CONTEXT, DEBUGGING_OPENGL);

        // Decide GL+GLSL versions
        glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 4);
        glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 5);
        // sample 4 times to enable MSAA Antialiasing
        glfwWindowHint(GLFW_SAMPLES, 4);
        // open window in maxed version so one can see the bottom right away
        glfwWindowHint(GLFW_MAXIMIZED, GLFW_TRUE);
        const GLFWvidmode* mode = glfwGetVideoMode(glfwGetPrimaryMonitor());

        // Create window with graphics context
        window = glfwCreateWindow(mode->width, mode->height, "Luhviz", nullptr, nullptr);
        if (window == nullptr) {
            LOG_ERROR(LOGGER, "could not create glfw window");
            return;
        }
        glfwMakeContextCurrent(window);
        int swap_interval = this->proxy.getConfigBool(luhviz_config_name, "enable_vsync") ? 1 : 0;
        glfwSwapInterval(swap_interval);  // Enable vsync (= 1)
        // glad: load all OpenGL function pointers
        // ---------------------------------------
        if (!gladLoadGLLoader((GLADloadproc)(glfwGetProcAddress))) {
            LOG_ERROR(LOGGER, "failed to initialise glad");
            return;
        }

        // create opengl context
        this->context = std::make_unique<GLContext>();
        this->context->create();

        // enable DEBUG
        this->context->enableDebugging(DEBUGGING_OPENGL);

        // set window icon
        GLFWimage images[4];
        std::unique_ptr<GLTexture> window_icon_100 = std::make_unique<GLTexture>();
        window_icon_100->create("res/images/luhbots_logo_100.png", false, true);
        images[0].width = window_icon_100->getWidth();
        images[0].height = window_icon_100->getHeight();
        images[0].pixels = window_icon_100->getImageData();
        std::unique_ptr<GLTexture> window_icon_80 = std::make_unique<GLTexture>();
        window_icon_80->create("res/images/luhbots_logo_80.png", false, true);
        images[1].width = window_icon_80->getWidth();
        images[1].height = window_icon_80->getHeight();
        images[1].pixels = window_icon_80->getImageData();
        std::unique_ptr<GLTexture> window_icon_50 = std::make_unique<GLTexture>();
        window_icon_50->create("res/images/luhbots_logo_50.png", false, true);
        images[2].width = window_icon_50->getWidth();
        images[2].height = window_icon_50->getHeight();
        images[2].pixels = window_icon_50->getImageData();
        std::unique_ptr<GLTexture> window_icon_30 = std::make_unique<GLTexture>();
        window_icon_30->create("res/images/luhbots_logo_30.png", false, true);
        images[3].width = window_icon_30->getWidth();
        images[3].height = window_icon_30->getHeight();
        images[3].pixels = window_icon_30->getImageData();
        glfwSetWindowIcon(window, 4, images);

        // create modules
        this->main_window = std::make_unique<MainWindow>(proxy);
        this->debugger = std::make_unique<Debugger>(fonts, proxy);
        this->inspector = std::make_unique<Inspector>(fonts, proxy);
        this->game_info = std::make_unique<GameInfo>(proxy, fonts);
        this->render_view = std::make_unique<RenderView>(this->marker_service, proxy, context);
        this->config = std::make_unique<LuhvizConfig>(proxy);
        this->skill_tester = std::make_unique<SkillTester>(proxy, fonts);
        this->skill_wizard = std::make_unique<SkillWizard>();
        this->robot_controller = std::make_unique<RobotController>(proxy);
        this->robot_info_display = std::make_unique<RobotInfoDisplay>(proxy);
        this->info_display = std::make_unique<InfoDisplay>(proxy);
        this->plotter = std::make_unique<Plotter>(proxy);
        this->gc_window = std::make_unique<GameControllerWindow>(proxy);

        std::unique_ptr<GLBuffer> vertex_buffer = std::make_unique<GLBuffer>();
        std::unique_ptr<GLBuffer> uv_buffer = std::make_unique<GLBuffer>();
        std::unique_ptr<GLShaderProgram> program = std::make_unique<GLShaderProgram>();
        std::unique_ptr<GLTexture> splashscreen = std::make_unique<GLTexture>();
        std::unique_ptr<GLVertexArray> vertex_array = std::make_unique<GLVertexArray>();
        showSplashScreen(vertex_buffer, uv_buffer, program, splashscreen, vertex_array);

        // init ImGui Render classes
        std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();
        render_view->init(this->window);
        main_window->init();
        debugger->init();
        inspector->init();
        game_info->init();
        config->init();
        skill_tester->init();
        skill_wizard->init();
        robot_controller->init();
        robot_info_display->init();
        info_display->init();
        plotter->init();
        gc_window->init();

        using namespace std::this_thread;  // sleep_for, sleep_until
        using namespace std::chrono;       // nanoseconds, system_clock, seconds
        steady_clock::time_point end = steady_clock::now();
        constexpr long long MIN_LOAD_TIME = 500;
        long long load_time = duration_cast<milliseconds>(end - begin).count();
        if (load_time < MIN_LOAD_TIME) {
            long long difference = MIN_LOAD_TIME - load_time;
            sleep_for(milliseconds(difference));
        }

        clearSplashScreen(vertex_buffer, uv_buffer, program, splashscreen, vertex_array);
        LOG_INFO(LOGGER, "luhviz initialized after {} ms loading time", load_time);
    }

    void stop() {
        if (!exit && enable) {
            glfwSetWindowShouldClose(window, GLFW_TRUE);
        }
    }

    void runLuhviz(std::atomic_bool& should_run) {
        if (exit) return;

        // Option to show the imgui demo window for testing purposes
        bool show_demo_window = false;
        //

        // Setup Dear ImGui context
        IMGUI_CHECKVERSION();
        ImGui::CreateContext();
        ImPlot::CreateContext();
        ImGuiIO& io = ImGui::GetIO();

        io.ConfigFlags |= ImGuiConfigFlags_NavEnableKeyboard;  // Enable Keyboard Controls
        io.ConfigFlags |= ImGuiConfigFlags_DockingEnable;      // Enable Docking
        bool enable_multiple_windows = this->proxy.getConfigBool("luhviz", "enable_multiple_windows");
        if (enable_multiple_windows) {
            io.ConfigFlags |= ImGuiConfigFlags_ViewportsEnable;  // Enable Multi-Viewport / Platform Windows
        }
        io.ConfigViewportsNoTaskBarIcon = true;
        config_flags = io.ConfigFlags;

        // take care of window layout saving and using the default one as fallback
        std::ifstream f(window_layout_file);
        // if layout file is not present, copy the default one to the directorys
        if (!f.good()) {
            auto fs = cmrc::luhviz::get_filesystem();
            auto layout = fs.open(default_window_layout_path);
            std::string default_layout{layout.begin(), layout.end()};
            std::ofstream out(window_layout_file);
            out << default_layout;
            out.close();

            LOG_INFO(LOGGER, "luhviz window layout .ini not found, using the default one as fallback");
        }
        io.IniFilename = window_layout_file.c_str();

        // Setup Dear ImGui style
        ImGui::StyleColorsDark();

        // When viewports are enabled we tweak WindowRounding/WindowBg so platform windows can look identical to regular
        // ones.
        ImGuiStyle& style = ImGui::GetStyle();
        if (io.ConfigFlags & ImGuiConfigFlags_ViewportsEnable) {
            style.WindowRounding = 0.0f;
            style.Colors[ImGuiCol_WindowBg].w = 1.0f;
        }
        bool team_blue = config_provider::ConfigProvider::getConfigStore().game_config.is_blue;
        setupImGuiStyle(team_blue);

        // Setup Platform/Renderer backends
        const std::string glsl_version = "#version 330 core";
        ImGui_ImplGlfw_InitForOpenGL(window, true);
        ImGui_ImplOpenGL3_Init(glsl_version.c_str());

        // load different fonts
        fonts.loadFonts();

        // fps computation
        double last_t = glfwGetTime();
        int nb_frames = 0;

        constexpr float CC = 0.15f;
        this->context->clearColor(CC, CC, CC, 1.0f);

        bool TEST_MODE = false;
        marker::MarkerTest test{this->marker_service, this->gdp};

        // Main loop
        ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2(0, 0));
        while (!glfwWindowShouldClose(window) && should_run) {
            // clear screen
            this->context->clear();

            // Start the Dear ImGui frame
            ImGui_ImplOpenGL3_NewFrame();
            ImGui_ImplGlfw_NewFrame();
            ImGui::NewFrame();

            if (team_blue != config_provider::ConfigProvider::getConfigStore().game_config.is_blue) {
                team_blue = config_provider::ConfigProvider::getConfigStore().game_config.is_blue;
                setupImGuiStyle(team_blue);
            }

            // show demo window of imgui, else draw luhviz
            if (show_demo_window) {
                ImGui::ShowDemoWindow(&show_demo_window);
                ImPlot::ShowDemoWindow();
            } else {
                if (TEST_MODE) {
                    test.displayTestMarkers();
                    // TEST_MODE = false;
                }

                main_window->render(last_fps, parameter_settings_open, skill_wizard_open, robot_controller_open,
                                    game_controller_open);

                // get copy of all markers from marker_service and filter them by the selected checkboxes in the
                // inspector
                auto luhviz_markers = marker_service.getLuhvizMarkers();
                inspector->createMarkerNs(*luhviz_markers);
                styleWindow();
                inspector->render();
                inspector->filterMarkers(*luhviz_markers);

                // give the filtered markers to the renderview
                render_view->update(*luhviz_markers);
                render_view->render();
                styleWindow();
                render_view->renderToTexture();

                styleWindow();
                game_info->render();

                styleWindow();
                debugger->render();

                styleWindow();
                config->render(&parameter_settings_open);

                styleWindow();
                skill_tester->render();

                styleWindow();
                skill_wizard->render(&skill_wizard_open);

                styleWindow();
                robot_controller->render(&robot_controller_open, this->active_controllers);

                styleWindow();
                robot_info_display->render(luhviz_markers->robot_info_markers);

                styleWindow();
                info_display->render(luhviz_markers->info_markers);

                styleWindow();
                plotter->render(luhviz_markers->plots);

                styleWindow();
                gc_window->render(&game_controller_open);

                proxy.update();
            }

            // close with exit
            if (ImGui::IsKeyDown(ImGuiKey_LeftCtrl) && ImGui::IsKeyDown(ImGuiKey_Q)) {
                glfwSetWindowShouldClose(window, GLFW_TRUE);
            }

            // ImGui Rendering
            ImGui::Render();
            ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());

            // Update and Render additional Platform Windows
            // (Platform functions may change the current OpenGL context, so we save/restore it to make it easier to
            // paste this code elsewhere.
            //  For this specific demo app we could also call glfwMakeContextCurrent(window) directly)
            if (this->config_flags & ImGuiConfigFlags_ViewportsEnable) {
                GLFWwindow* backup_current_context = glfwGetCurrentContext();
                ImGui::UpdatePlatformWindows();
                ImGui::RenderPlatformWindowsDefault();
                glfwMakeContextCurrent(backup_current_context);
            }

            double current_time = glfwGetTime();

            // handle gamepad input
            this->handleControllerInput(current_time);

            glfwSwapBuffers(window);
            // Poll and handle events (inputs, window resize, etc.)
            glfwPollEvents();

            // independent calculate the actual fps
            // double current_time = glfwGetTime();
            nb_frames++;
            if (current_time - last_t >= 1.0) {
                last_fps = nb_frames;
                nb_frames = 0;
                last_t += 1.0;

                // update dataproxy states for dropdown / selections
                proxy.pollSelections();
            }
        }
        ImGui::PopStyleVar();
    }

    void loop(std::atomic_bool& should_run) {
        if (should_run && this->enable) {
            runLuhviz(should_run);
        } else {
            while (should_run) {
                // Don't busy wait on this thread. Give the cpu time to others
                constexpr unsigned int SLEEP_TIME_MS = 500;
                std::this_thread::sleep_for(std::chrono::milliseconds(SLEEP_TIME_MS));
            }
        }
    }

    void styleWindow() {
        ImGuiWindowClass window_class;
        window_class.DockNodeFlagsOverrideSet = ImGuiDockNodeFlags_NoWindowMenuButton;
        ImGui::SetNextWindowClass(&window_class);
    }

    void showSplashScreen(std::unique_ptr<GLBuffer>& vertex_buffer, std::unique_ptr<GLBuffer>& uv_buffer,
                          std::unique_ptr<GLShaderProgram>& program, std::unique_ptr<GLTexture>& splashscreen,
                          std::unique_ptr<GLVertexArray>& vao) {
        splashscreen->create("res/images/splashscreen.png", true);

        // Create and compile our GLSL program from the shaders
        const std::string ver_shader_textured = "res/shader/simpleTextureVertexShader.glsl";
        const std::string frag_shader_textured = "res/shader/simpleTextureFragmentShader.glsl";
        program->create(ver_shader_textured, frag_shader_textured);
        this->context->useProgram(*program);

        GLUniform1i sampler_id{program->getId(), "myTextureSampler"};

        const std::vector<glm::vec3> g_vertex_buffer_data{
            {-1.0f, -1.0f, 0.0f}, {1.0f, -1.0f, 0.0f}, {1.0f, 1.0f, 0.0f},
            {-1.0f, -1.0f, 0.0f}, {1.0f, 1.0f, 0.0f},  {-1.0f, 1.0f, 0.0f},
        };
        const std::vector<glm::vec2> g_uv_buffer_data{{0.0f, 0.0f}, {1.0f, 0.0f}, {1.0f, 1.0f},
                                                      {0.0f, 0.0f}, {1.0f, 1.0f}, {0.0f, 1.0f}};

        this->context->clear();

        vao->bind();

        vertex_buffer->bufferArrayDataVert(g_vertex_buffer_data);
        vao->bindAttribute(0, 3, nullptr, sizeof(glm::vec3));

        uv_buffer->bufferArrayDataUV(g_uv_buffer_data);
        vao->bindAttribute(1, 2, nullptr, sizeof(glm::vec2));

        this->context->bindTexture(*splashscreen, 0);
        sampler_id.set(0);

        const int num_verts = 6;
        this->context->drawArrays(*vao, num_verts);

        // Swap buffers
        glfwSwapBuffers(window);
        glfwPollEvents();
    }

    void clearSplashScreen(std::unique_ptr<GLBuffer>& vertex_buffer, std::unique_ptr<GLBuffer>& uv_buffer,
                           std::unique_ptr<GLShaderProgram>& program, std::unique_ptr<GLTexture>& splashscreen,
                           std::unique_ptr<GLVertexArray>& vertex_array) {
        vertex_buffer.reset();
        uv_buffer.reset();
        splashscreen.reset();
        vertex_array.reset();
        program.reset();
    }

    // NOLINTBEGIN(cppcoreguidelines-avoid-magic-numbers)
    void setupImGuiStyle(bool team_blue) {
        // Rounded Visual Studio style by RedNicStone from ImThemes
        ImGuiStyle& style = ImGui::GetStyle();

        // style.Alpha = 1.0f;
        // style.DisabledAlpha = 0.6000000238418579f;
        // style.WindowPadding = ImVec2(8.0f, 8.0f);
        // style.WindowRounding = 4.0f;
        // style.WindowBorderSize = 1.5f;
        // style.WindowMinSize = ImVec2(32.0f, 32.0f);
        // style.WindowTitleAlign = ImVec2(0.0f, 0.5f);
        // style.WindowMenuButtonPosition = ImGuiDir_Left;
        // style.ChildRounding = 0.0f;
        // style.ChildBorderSize = 1.0f;
        // style.PopupRounding = 4.0f;
        // style.PopupBorderSize = 1.0f;
        // style.FramePadding = ImVec2(4.0f, 3.0f);
        // style.FrameRounding = 2.5f;
        // style.FrameBorderSize = 0.0f;
        // style.ItemSpacing = ImVec2(8.0f, 4.0f);
        // style.ItemInnerSpacing = ImVec2(4.0f, 4.0f);
        // style.CellPadding = ImVec2(4.0f, 2.0f);
        // style.IndentSpacing = 21.0f;
        // style.ColumnsMinSpacing = 6.0f;
        // style.ScrollbarSize = 11.0f;
        // style.ScrollbarRounding = 2.5f;
        // style.GrabMinSize = 10.0f;
        // style.GrabRounding = 2.0f;
        // style.TabRounding = 3.5f;
        // style.TabBorderSize = 0.0f;
        // style.TabMinWidthForCloseButton = 0.0f;
        // style.ColorButtonPosition = ImGuiDir_Right;
        // style.ButtonTextAlign = ImVec2(0.5f, 0.5f);
        // style.SelectableTextAlign = ImVec2(0.0f, 0.0f);

        style.Colors[ImGuiCol_Text] = ImVec4(1.0f, 1.0f, 1.0f, 1.0f);
        style.Colors[ImGuiCol_TextDisabled] =
            ImVec4(0.5921568870544434f, 0.5921568870544434f, 0.5921568870544434f, 1.0f);
        style.Colors[ImGuiCol_WindowBg] = ImVec4(0.1450980454683304f, 0.1450980454683304f, 0.1490196138620377f, 1.0f);
        style.Colors[ImGuiCol_ChildBg] = ImVec4(0.1450980454683304f, 0.1450980454683304f, 0.1490196138620377f, 1.0f);
        style.Colors[ImGuiCol_PopupBg] = ImVec4(0.1450980454683304f, 0.1450980454683304f, 0.1490196138620377f, 1.0f);
        style.Colors[ImGuiCol_Border] = ImVec4(0.3058823645114899f, 0.3058823645114899f, 0.3058823645114899f, 1.0f);
        style.Colors[ImGuiCol_BorderShadow] =
            ImVec4(0.3058823645114899f, 0.3058823645114899f, 0.3058823645114899f, 1.0f);
        style.Colors[ImGuiCol_FrameBg] = ImVec4(0.2000000029802322f, 0.2000000029802322f, 0.2156862765550613f, 1.0f);
        style.Colors[ImGuiCol_FrameBgHovered] = ImVec4(0 / 255.0, 72 / 255.0, 128 / 255.0, 1.0f);
        style.Colors[ImGuiCol_FrameBgActive] = ImVec4(0 / 255.0, 0 / 255.0, 0 / 255.0, 1.0f);
        style.Colors[ImGuiCol_TitleBg] = ImVec4(0.1450980454683304f, 0.1450980454683304f, 0.1490196138620377f, 1.0f);
        style.Colors[ImGuiCol_TitleBgActive] =
            ImVec4(0.1450980454683304f, 0.1450980454683304f, 0.1490196138620377f, 1.0f);
        style.Colors[ImGuiCol_TitleBgCollapsed] =
            ImVec4(0.1450980454683304f, 0.1450980454683304f, 0.1490196138620377f, 1.0f);
        style.Colors[ImGuiCol_MenuBarBg] = ImVec4(0.2000000029802322f, 0.2000000029802322f, 0.2156862765550613f, 1.0f);
        style.Colors[ImGuiCol_ScrollbarBg] =
            ImVec4(0.2000000029802322f, 0.2000000029802322f, 0.2156862765550613f, 1.0f);
        style.Colors[ImGuiCol_ScrollbarGrab] =
            ImVec4(0.321568638086319f, 0.321568638086319f, 0.3333333432674408f, 1.0f);
        style.Colors[ImGuiCol_ScrollbarGrabHovered] =
            ImVec4(0.2000000029802322f, 0.2000000029802322f, 0.2156862765550613f, 1.0f);
        style.Colors[ImGuiCol_ScrollbarGrabActive] =
            ImVec4(0.3529411852359772f, 0.3529411852359772f, 0.3725490272045135f, 1.0f);
        style.Colors[ImGuiCol_Button] = ImVec4(0.2000000029802322f, 0.2000000029802322f, 0.2156862765550613f, 1.0f);
        style.Colors[ImGuiCol_ButtonHovered] =
            ImVec4(0.1137254908680916f, 0.5921568870544434f, 0.9254902005195618f, 1.0f);
        style.Colors[ImGuiCol_ButtonActive] =
            ImVec4(0.1137254908680916f, 0.5921568870544434f, 0.9254902005195618f, 1.0f);
        style.Colors[ImGuiCol_Header] = ImVec4(0.2000000029802322f, 0.2000000029802322f, 0.2156862765550613f, 1.0f);
        style.Colors[ImGuiCol_HeaderHovered] =
            ImVec4(0.1137254908680916f, 0.5921568870544434f, 0.9254902005195618f, 1.0f);
        style.Colors[ImGuiCol_HeaderActive] = ImVec4(0.0f, 0.4666666686534882f, 0.7843137383460999f, 1.0f);
        style.Colors[ImGuiCol_Separator] = ImVec4(0.3058823645114899f, 0.3058823645114899f, 0.3058823645114899f, 1.0f);
        style.Colors[ImGuiCol_SeparatorHovered] =
            ImVec4(0.3058823645114899f, 0.3058823645114899f, 0.3058823645114899f, 1.0f);
        style.Colors[ImGuiCol_SeparatorActive] =
            ImVec4(0.3058823645114899f, 0.3058823645114899f, 0.3058823645114899f, 1.0f);
        style.Colors[ImGuiCol_ResizeGrip] = ImVec4(0.1450980454683304f, 0.1450980454683304f, 0.1490196138620377f, 1.0f);
        style.Colors[ImGuiCol_ResizeGripHovered] =
            ImVec4(0.2000000029802322f, 0.2000000029802322f, 0.2156862765550613f, 1.0f);
        style.Colors[ImGuiCol_ResizeGripActive] =
            ImVec4(0.321568638086319f, 0.321568638086319f, 0.3333333432674408f, 1.0f);

        style.Colors[ImGuiCol_TabUnfocused] =
            ImVec4(0.1450980454683304f, 0.1450980454683304f, 0.1490196138620377f, 1.0f);
        if (team_blue) {
            style.Colors[ImGuiCol_Tab] = ImVec4(0.1450980454683304f, 0.1450980454683304f, 0.1490196138620377f, 1.0f);
            style.Colors[ImGuiCol_TabHovered] =
                ImVec4(0.1137254908680916f, 0.5921568870544434f, 0.9254902005195618f, 1.0f);
            style.Colors[ImGuiCol_TabActive] = ImVec4(0.0f, 0.4666666686534882f, 0.7843137383460999f, 1.0f);
            style.Colors[ImGuiCol_TabUnfocusedActive] = ImVec4(0.0f, 0.4666666686534882f, 0.7843137383460999f, 1.0f);
            style.Colors[ImGuiCol_CheckMark] = ImVec4(0.0f, 0.4666666686534882f, 0.7843137383460999f, 1.0f);
            style.Colors[ImGuiCol_SliderGrab] =
                ImVec4(0.1137254908680916f, 0.5921568870544434f, 0.9254902005195618f, 1.0f);
            style.Colors[ImGuiCol_SliderGrabActive] = ImVec4(0.0f, 0.4666666686534882f, 0.7843137383460999f, 1.0f);
            proxy.accent_text_color = ImVec4{1, 1, 1, 1};
        } else {
            style.Colors[ImGuiCol_Tab] = ImVec4(0.6, 0.6f, 0.0f, 1.0);
            style.Colors[ImGuiCol_TabHovered] = ImVec4(1.0f, 1.0f, 0.0f, 1.0f);
            style.Colors[ImGuiCol_TabActive] = ImVec4(1.0f, 1.0f, 0.0f, 1.0f);
            style.Colors[ImGuiCol_TabUnfocusedActive] = ImVec4(1.0f, 1.0f, 0.0f, 1.0f);
            style.Colors[ImGuiCol_TabUnfocused] = ImVec4(0.6f, 0.6f, 0.0f, 1.0f);
            style.Colors[ImGuiCol_CheckMark] = ImVec4(0.8f, 0.8f, 0.0f, 1.0f);
            style.Colors[ImGuiCol_SliderGrab] = ImVec4(0.8f, 0.8f, 0.0f, 1.0f);
            style.Colors[ImGuiCol_SliderGrabActive] = ImVec4(1.0f, 1.0f, 0.0f, 1.0f);
            proxy.accent_text_color = ImVec4{0, 0, 0, 1};
        }

        style.Colors[ImGuiCol_PlotLines] = ImVec4(0.0f, 0.4666666686534882f, 0.7843137383460999f, 1.0f);
        style.Colors[ImGuiCol_PlotLinesHovered] =
            ImVec4(0.1137254908680916f, 0.5921568870544434f, 0.9254902005195618f, 1.0f);
        style.Colors[ImGuiCol_PlotHistogram] = ImVec4(0.0f, 0.4666666686534882f, 0.7843137383460999f, 1.0f);
        style.Colors[ImGuiCol_PlotHistogramHovered] =
            ImVec4(0.1137254908680916f, 0.5921568870544434f, 0.9254902005195618f, 1.0f);
        style.Colors[ImGuiCol_TableHeaderBg] =
            ImVec4(0.1882352977991104f, 0.1882352977991104f, 0.2000000029802322f, 1.0f);
        style.Colors[ImGuiCol_TableBorderStrong] =
            ImVec4(0.3098039329051971f, 0.3098039329051971f, 0.3490196168422699f, 1.0f);
        style.Colors[ImGuiCol_TableBorderLight] =
            ImVec4(0.2274509817361832f, 0.2274509817361832f, 0.2470588237047195f, 1.0f);
        style.Colors[ImGuiCol_TableRowBg] = ImVec4(0.0f, 0.0f, 0.0f, 0.0f);
        style.Colors[ImGuiCol_TableRowBgAlt] = ImVec4(1.0f, 1.0f, 1.0f, 0.05999999865889549f);
        style.Colors[ImGuiCol_TextSelectedBg] = ImVec4(0.0f, 0.4666666686534882f, 0.0, 1.0f);
        style.Colors[ImGuiCol_DragDropTarget] =
            ImVec4(0.1450980454683304f, 0.1450980454683304f, 0.1490196138620377f, 1.0f);
        style.Colors[ImGuiCol_NavHighlight] =
            ImVec4(0.1450980454683304f, 0.1450980454683304f, 0.1490196138620377f, 1.0f);
        style.Colors[ImGuiCol_NavWindowingHighlight] = ImVec4(1.0f, 1.0f, 1.0f, 0.699999988079071f);
        style.Colors[ImGuiCol_NavWindowingDimBg] =
            ImVec4(0.800000011920929f, 0.800000011920929f, 0.800000011920929f, 0.2000000029802322f);
        style.Colors[ImGuiCol_ModalWindowDimBg] =
            ImVec4(0.1450980454683304f, 0.1450980454683304f, 0.1490196138620377f, 1.0f);
    }
    // NOLINTEND

    ~LuhvizMainImpl() {
        if (this->enable) {
            // Cleanup VBO and shader
            render_view->saveAndCleanup();
            config->saveAtClose();

            render_view.reset();
            main_window.reset();
            debugger.reset();
            inspector.reset();
            game_info.reset();
            config.reset();
            skill_tester.reset();
            skill_wizard.reset();
            robot_controller.reset();
            robot_info_display.reset();
            this->context.reset();

            // Cleanup
            ImGui_ImplOpenGL3_Shutdown();
            ImGui_ImplGlfw_Shutdown();
            ImPlot::DestroyContext();
            ImGui::DestroyContext();

            glfwDestroyWindow(window);
            glfwTerminate();
        }
        exit = true;
    }

   private:
    luhsoccer::logger::Logger logger{"luhviz/luhviz_main"};

    // module references
    luhsoccer::marker::MarkerService& marker_service;
    luhsoccer::ssl_interface::SSLInterface& ssl;
    simulation_interface::SimulationInterface& sim;
    luhsoccer::game_data_provider::GameDataProvider& gdp;
    local_planner::LocalPlannerModule& local_planner;
    skills::BodSkillBook& skill_book;
    robot_interface::RobotInterface& robot_interface;
    scenario::ScenarioExecutor& scenario_executor;

    constexpr static int WINDOW_WIDTH = 1280;
    constexpr static int WINDOW_HEIGTH = 720;
    const std::string luhviz_config_name = "luhviz";
    const std::string window_layout_file = "./luhviz.ini";
    const std::string default_window_layout_path = "res/layout/luhviz_default.ini";

    ImGuiConfigFlags config_flags{};
    int last_fps{0};
    bool exit{false};
    bool enable{false};

    Fonts fonts{};

    std::unique_ptr<GLContext> context;
    luhsoccer::luhviz::DataProxy proxy{this->ssl,
                                       this->sim,
                                       this->local_planner,
                                       this->skill_book,
                                       this->robot_interface,
                                       this->gdp,
                                       this->scenario_executor};
    std::unique_ptr<MainWindow> main_window;
    std::unique_ptr<Debugger> debugger;
    std::unique_ptr<Inspector> inspector;
    std::unique_ptr<GameInfo> game_info;
    std::unique_ptr<RenderView> render_view;
    std::unique_ptr<LuhvizConfig> config;
    std::unique_ptr<SkillTester> skill_tester;
    std::unique_ptr<SkillWizard> skill_wizard;
    std::unique_ptr<RobotController> robot_controller;
    std::unique_ptr<RobotInfoDisplay> robot_info_display;
    std::unique_ptr<InfoDisplay> info_display;
    std::unique_ptr<Plotter> plotter;
    std::unique_ptr<GameControllerWindow> gc_window;

    bool parameter_settings_open = false;
    bool skill_wizard_open = false;
    bool robot_controller_open = false;
    bool game_controller_open = false;

    GLFWwindow* window{};

    std::vector<size_t> active_controllers{};

    void handleControllerInput(double current_time) {
        // get current active count
        this->active_controllers.clear();
        for (size_t i = GLFW_JOYSTICK_1; i <= GLFW_JOYSTICK_LAST; ++i) {
            if (glfwJoystickIsGamepad(i)) {
                active_controllers.emplace_back(i);
            }
        }

        // statusbar
        if (active_controllers.size() > 1) {
            proxy.setGamepadStatus(GamepadStatus::CONNECTED);
        } else {
            proxy.setGamepadStatus(GamepadStatus::DISSCONECTED);
        }

        // controller input
        for (const size_t& conti : active_controllers) {
            GLFWgamepadstate state;

            if (glfwGetGamepadState(conti, &state)) {
                if (state.buttons[GLFW_GAMEPAD_BUTTON_X]) {
                    proxy.gamepadButtonInput(GamepadControls::BUTTON_KICKER, GLFW_PRESS, conti, current_time);
                }

                if (state.buttons[GLFW_GAMEPAD_BUTTON_B]) {
                    proxy.gamepadButtonInput(GamepadControls::BUTTON_DRIBBLER, GLFW_PRESS, conti, current_time);
                } else if (!state.buttons[GLFW_GAMEPAD_BUTTON_B]) {
                    proxy.gamepadButtonInput(GamepadControls::BUTTON_DRIBBLER, GLFW_RELEASE, conti, current_time);
                }

                if (state.buttons[GLFW_GAMEPAD_BUTTON_DPAD_UP]) {
                    proxy.gamepadButtonInput(GamepadControls::BUTTON_VOLTAGE_UP, GLFW_PRESS, conti, current_time);
                } else if (!state.buttons[GLFW_GAMEPAD_BUTTON_DPAD_UP]) {
                    proxy.gamepadButtonInput(GamepadControls::BUTTON_VOLTAGE_UP, GLFW_RELEASE, conti, current_time);
                }

                if (state.buttons[GLFW_GAMEPAD_BUTTON_DPAD_DOWN]) {
                    proxy.gamepadButtonInput(GamepadControls::BUTTON_VOLTAGE_DOWN, GLFW_PRESS, conti, current_time);
                } else if (!state.buttons[GLFW_GAMEPAD_BUTTON_DPAD_DOWN]) {
                    proxy.gamepadButtonInput(GamepadControls::BUTTON_VOLTAGE_DOWN, GLFW_RELEASE, conti, current_time);
                }

                // handle axes
                float x = state.axes[GLFW_GAMEPAD_AXIS_LEFT_X];
                float y = -state.axes[GLFW_GAMEPAD_AXIS_LEFT_Y];
                float trigger_left = (state.axes[GLFW_GAMEPAD_AXIS_LEFT_TRIGGER] + 1) * 2.0f;
                float trigger_right = (state.axes[GLFW_GAMEPAD_AXIS_RIGHT_TRIGGER] + 1) * 2.0f;

                // LOG_WARNING(LOGGER, trigger_left - trigger_right);

                proxy.gamepadAxesInput(x, y, trigger_left - trigger_right, conti,
                                       this->robot_controller->isGlobalSteering());
            }
        }

        // add controll for keyboard
        active_controllers.emplace_back(GLFW_JOYSTICK_LAST + 1);

        // keyboard control
        int state_up = glfwGetKey(window, GLFW_KEY_W);
        int state_down = glfwGetKey(window, GLFW_KEY_S);
        int state_left = glfwGetKey(window, GLFW_KEY_A);
        int state_right = glfwGetKey(window, GLFW_KEY_D);
        int state_left_rot = glfwGetKey(window, GLFW_KEY_Q);
        int state_right_rot = glfwGetKey(window, GLFW_KEY_E);

        int state_voltage_up = glfwGetKey(window, GLFW_KEY_UP);
        int state_voltage_down = glfwGetKey(window, GLFW_KEY_DOWN);

        int state_kick = glfwGetKey(window, GLFW_KEY_ENTER);
        int state_dribbler = glfwGetKey(window, GLFW_KEY_SPACE);

        float x = -(state_left - state_right);
        float y = (state_up - state_down);

        size_t keyboard_id = GLFW_JOYSTICK_LAST + 1;

        proxy.gamepadAxesInput(x, y, state_left_rot - state_right_rot, keyboard_id,
                               this->robot_controller->isGlobalSteering());

        if (state_kick) {
            proxy.gamepadButtonInput(GamepadControls::BUTTON_KICKER, GLFW_PRESS, keyboard_id, current_time);
        }

        if (state_dribbler) {
            proxy.gamepadButtonInput(GamepadControls::BUTTON_DRIBBLER, GLFW_PRESS, keyboard_id, current_time);
        } else if (!state_dribbler) {
            proxy.gamepadButtonInput(GamepadControls::BUTTON_DRIBBLER, GLFW_RELEASE, keyboard_id, current_time);
        }

        if (state_voltage_up) {
            proxy.gamepadButtonInput(GamepadControls::BUTTON_VOLTAGE_UP, GLFW_PRESS, keyboard_id, current_time);
        } else if (!state_voltage_up) {
            proxy.gamepadButtonInput(GamepadControls::BUTTON_VOLTAGE_UP, GLFW_RELEASE, keyboard_id, current_time);
        }

        if (state_voltage_down) {
            proxy.gamepadButtonInput(GamepadControls::BUTTON_VOLTAGE_DOWN, GLFW_PRESS, keyboard_id, current_time);
        } else if (!state_voltage_down) {
            proxy.gamepadButtonInput(GamepadControls::BUTTON_VOLTAGE_DOWN, GLFW_RELEASE, keyboard_id, current_time);
        }

        proxy.publishRobotData(current_time);
    }
};

LuhvizMain::LuhvizMain(marker::MarkerService& ms, ssl_interface::SSLInterface& ssl,
                       simulation_interface::SimulationInterface& sim, game_data_provider::GameDataProvider& gdp,
                       local_planner::LocalPlannerModule& local_planner, skills::BodSkillBook& skill_book,
                       robot_interface::RobotInterface& robot_interface, scenario::ScenarioExecutor& scenario_executor)
    : implementation(std::make_unique<LuhvizMainImpl>(ms, ssl, sim, gdp, local_planner, skill_book, robot_interface,
                                                      scenario_executor)){};
LuhvizMain::~LuhvizMain() = default;

void LuhvizMain::setup() { this->implementation->setup(); }
void LuhvizMain::loop(std::atomic_bool& should_run) { this->implementation->loop(should_run); }
void LuhvizMain::stop() { this->implementation->stop(); }

}  // namespace luhsoccer::luhviz
