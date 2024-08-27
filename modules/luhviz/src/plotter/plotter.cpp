#include "plotter/include/plotter.hpp"

namespace luhsoccer::luhviz {

void Plotter::init() {}

void Plotter::render(std::unordered_map<std::string, marker::LinePlot>& line_plots, bool& open) {
    if (!open) {
        return;
    }

    ImGui::PushStyleColor(ImGuiCol_Text, this->proxy.accent_text_color);
    ImGui::Begin("Plotter", &open);
    ImGui::PopStyleColor();

    const ImPlotAxisFlags flags_x = ImPlotAxisFlags_NoTickLabels;
    const ImPlotAxisFlags flags_y = ImPlotAxisFlags_AutoFit;

    static float history = 10.0f;
    ImGui::SliderFloat("Zoom", &history, 1, 30, "%.1f s");

    float t = time::timeSinceStart().asSec();

    for (const auto& [id, plot] : line_plots) {
        const float left = plot.getLeftLimit();
        const float right = plot.getRightLimit();
        if (ImPlot::BeginPlot(("##" + (std::string)plot).c_str(), ImVec2(-1, 250))) {
            ImPlot::SetupAxes(nullptr, nullptr, flags_x, flags_y);
            ImPlot::SetupAxisLimits(ImAxis_X1, left, right, ImGuiCond_Always);
            ImPlot::SetupAxisLimits(ImAxis_Y1, 0, 1);
            ImPlot::SetNextFillStyle(IMPLOT_AUTO_COL, 0.5f);

            for (size_t i = 0; i < plot.getLineCount(); i++) {
                const marker::LinePlot::LineData& data = plot[i];
                const auto& x_data = data.getDataX();
                const auto& y_data = data.getDataY();
                const std::string& label = data.getLabel();
                ImPlot::PlotLine(label.c_str(), &x_data[0], &y_data[0], x_data.size());
            }
            ImPlot::EndPlot();
        }

        float window_width = ImGui::GetWindowSize().x;
        float text_width = ImGui::CalcTextSize(id.c_str()).x;
        float indentation = 0.5f * (window_width - text_width);

        ImVec2 pos = ImGui::GetCursorPos();
        ImGui::SetCursorPos({pos.x + indentation, pos.y});
        ImGui::Text("%s", id.c_str());
        ImGui::Spacing();
    }

    ImGui::End();
}

}  // namespace luhsoccer::luhviz
