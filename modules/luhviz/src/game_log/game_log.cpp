#include "include/game_log.hpp"
#include "utils/utils.hpp"

namespace luhsoccer::luhviz {
void GameLog::render(bool& open) {
    if (!open) {
        return;
    }

    ImGui::PushStyleColor(ImGuiCol_Text, this->proxy.accent_text_color);
    ImGui::Begin("Game-Log", &open);
    ImGui::PopStyleColor();

    if (ImGui::Button("Open Game-Log")) {
        auto files = openFile("Open Gamelog", "");
        if (files.size() > 0) {
            proxy.replayGamelog(std::move(files[0]));
        }
    }

    auto current_frame = proxy.getLogFileControlHandle().current_frame.load();

    bool no_log = false;

    switch (proxy.getLogFileControlHandle().state.load()) {
        case ssl_interface::LogFileState::NOT_LOADED:
            no_log = true;
            ImGui::Text("State: No Game-Log loaded!");
            break;
        case ssl_interface::LogFileState::PARSING:
            ImGui::Text("State: Parsing %c", "|/-\\"[(int)(ImGui::GetTime() / 0.05f) & 3]);
            break;
        case ssl_interface::LogFileState::RUNNING:
            ImGui::Text("State: Running");
            break;
        default:
            no_log = true;
            ImGui::Text("State: Unknown");
            break;
    }

    ImGui::BeginDisabled(no_log);

    ImGui::Text("Frame: %zu / %zu (%f %%)", current_frame, proxy.getLogFileControlHandle().max_frames.load(),
                (static_cast<float>(current_frame) /
                 static_cast<float>(this->proxy.getLogFileControlHandle().max_frames.load())) *
                    100.0f);

    ImGui::SliderInt("Replay Factor", &replay_factor, -10, 10);
    ImGui::Checkbox("10x", &x10);

    ImGui::SliderInt("Future path frames", &preload_paths, 0, 1000);

    if (preload_paths != last_preload_paths) {
        last_preload_paths = preload_paths;
        proxy.getLogFileControlHandle().preload_paths.store(preload_paths);
    }

    if (replay_factor != last_replay_factor) {
        last_replay_factor = replay_factor;
        if (x10) {
            last_replay_factor *= 10;
        }
        proxy.getLogFileControlHandle().replay_factor.store(last_replay_factor);
    }

    ImGui::EndDisabled();
    ImGui::End();
}
}  // namespace luhsoccer::luhviz