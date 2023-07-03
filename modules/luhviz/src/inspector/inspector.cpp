#include "include/inspector.hpp"
#include "marker_service/marker_2d_impl.hpp"
#include "marker_service/marker_impl.hpp"
#include "marker_service/luhviz_impl.hpp"

namespace luhsoccer::luhviz {

void Inspector::init() {
    // TODO: read the saved disabled namespaces and apply them at start
    // TODO: on exit, save the disabled namespaces
}

void Inspector::render() {
    ImGuiWindowFlags window_flags = ImGuiWindowFlags_NoCollapse | ImGuiWindowFlags_NoNavFocus |
                                    ImGuiWindowFlags_NoBringToFrontOnFocus | ImGuiWindowFlags_NoScrollbar |
                                    ImGuiWindowFlags_NoScrollWithMouse;

    ImGui::PushStyleColor(ImGuiCol_Text, this->data_proxy.accent_text_color);
    ImGui::Begin("Inspector", nullptr, window_flags);
    ImGui::PopStyleColor();

    // display a tree for every marker namespace
    ImGui::PushTextWrapPos(checkbox_xoffset - 30);
    // display the trees for the different marker namespaces
    size_t index = 0;
    for (auto& ns : this->marker_ns) {
        if (this->tree_expanded.find(ns.first) == this->tree_expanded.end()) {
            // add value if namespace is not known yet
            this->tree_expanded.emplace(ns.first, false);
        }
        if (this->checks.find(ns.first) == this->checks.end()) {
            // add value if namespace is not known yet
            this->checks.emplace(ns.first, true);
        }

        if (index >= item_checks.size()) {
            item_checks.emplace_back();
        }

        bool expanded = addTreeItemCheckbox(ns.first, index, this->checks[(ns.first)], ImGuiTreeNodeFlags_None);
        size_t i = 0;
        this->tree_expanded[ns.first] = true;
        for (auto& info : ns.second->marker_infos) {
            if (i >= this->item_checks[index].size()) {
                this->item_checks[index].emplace_back(true);
            }
            // display tree leaf for every marker in that namespace
            std::string leaf_id = info->ns + "/" + std::to_string(info->id);
            if (!info->name.empty()) leaf_id = info->ns + "/" + info->name;
            if (expanded) {
                addTreeItemCheckbox(leaf_id, i, this->item_checks[index][i], ImGuiTreeNodeFlags_Leaf);
                ImGui::TreePop();
            }
            // apply marker enabled/disabled
            if (index < this->item_checks.size() && i < this->item_checks[index].size()) {
                info->enabled = this->item_checks[index][i];
            }
            i++;
        }
        if (expanded) ImGui::TreePop();

        // apply ns enabled/disabled
        ns.second->enabled = this->checks[ns.first];

        ++index;
    }
    ImGui::PopTextWrapPos();

    checkbox_xoffset = ImGui::GetWindowWidth() - 40;

    ImGui::End();
}

bool Inspector::addTreeItemCheckbox(const std::string& text, size_t i, bool& checked, int flag) {
    bool expanded = ImGui::TreeNodeEx(("##" + text).c_str(), flag);
    ImGui::SameLine();
    ImGui::Text("%s", text.c_str());
    ImGui::SameLine();
    ImGui::SetCursorPosX(checkbox_xoffset);
    addCheckbox(text, i, checked);
    return expanded;
}

void Inspector::addCheckbox(const std::string& text, size_t i, bool& checked) {
    ImFont* small_font = fonts.getFont(fonts.FONT_SMALL);
    ImGui::PushFont(small_font);
    std::string id = "##check_" + text + std::to_string(i);
    ImGui::Checkbox(id.c_str(), &checked);
    ImGui::PopFont();
}

void Inspector::createMarkerNs(const marker::LuhvizMarkers& luhviz_markers) {
    // go through markers and set color alpha to 0 if needed
    // combine markers into one array with necessary infos
    this->marker_ns.clear();

    // 3D Markers
    int index1 = 0;
    for (const auto& m : luhviz_markers.markers) {
        auto new_info = std::make_shared<MarkerInfo>(m.second.getNs(), m.second.getId(), m.second.getType(), false,
                                                     index1, m.second.getText());

        if (this->marker_ns.contains(m.second.getNs())) {
            // if existing namespace, append the info to the vector
            auto it = this->marker_ns.find(m.second.getNs());
            it->second->marker_infos.emplace_back(new_info);
        } else {
            // else save new namespace
            auto new_marker_ns = std::make_shared<MarkerNs>();
            new_marker_ns->marker_infos.emplace_back(new_info);
            this->marker_ns.insert_or_assign(m.second.getNs(), new_marker_ns);
        }
        index1++;
    }
    // same for 2D markers
    int index2 = 0;
    for (const auto& m : luhviz_markers.markers2d) {
        auto new_info =
            std::make_shared<MarkerInfo>(m.second.getNs(), m.second.getId(), m.second.getType(), true, index2, "");

        if (this->marker_ns.contains(m.second.getNs())) {
            // if existing namespace, append the info to the vector
            auto it = this->marker_ns.find(m.second.getNs());
            it->second->marker_infos.emplace_back(new_info);
        } else {
            // else save new namespace
            auto new_marker_ns = std::make_shared<MarkerNs>();
            new_marker_ns->marker_infos.emplace_back(new_info);
            this->marker_ns.insert_or_assign(m.second.getNs(), new_marker_ns);
        }
        index2++;
    }
}

void Inspector::filterMarkers(marker::LuhvizMarkers& luhviz_markers) const {
    // remove markers from maps if they are not set enabled via checkboxes
    for (const auto& ns : this->marker_ns) {
        for (const auto& info : ns.second->marker_infos) {
            if (!ns.second->enabled || !info->enabled) {
                std::string key = info->ns + "_" + std::to_string(info->id);
                if (ns.first.compare("Frames") == 0 || ns.first.compare("Frame_names") == 0) {
                    key = info->ns + "_" + info->name;
                }
                if (info->is2d) {
                    luhviz_markers.markers2d.erase(key);
                } else {
                    luhviz_markers.markers.erase(key);
                }
            }
        }
    }
}
}  // namespace luhsoccer::luhviz