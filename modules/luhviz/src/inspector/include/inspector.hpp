#pragma once

#include <iostream>
#include <utility>
#include "imgui.h"
#include "marker_service/marker_service.hpp"
#include "common/include/fonts.hpp"
#include "config/config_store.hpp"
#include "include/data_proxy.hpp"

#include <deque>

namespace luhsoccer::luhviz {
struct MarkerInfo {
    MarkerInfo(std::string ns, size_t id, marker::MType type, bool is2d, int index, std::string name)
        : ns(std::move(ns)), name(std::move(name)), id(id), type(type), is2d(is2d), index(index) {}

    std::string ns{"default"};
    std::string name{""};
    std::string key{};
    size_t id{0};
    marker::MType type{marker::MType::LAST_MARKER_TYPE};
    bool is2d{false};
    int index{0};
    bool enabled{false};
};

struct MarkerNs {
    MarkerNs() = default;
    bool enabled{true};
    std::vector<std::shared_ptr<MarkerInfo>> marker_infos;
};

class Inspector {
   public:
    Inspector(Fonts& fonts, DataProxy& proxy) : data_proxy(proxy), fonts(fonts) {}
    // ----- members -----
    // ----- methods -----
    void init();
    void render();

    /**
     * @brief Create the marker namespaces structure from the luhviz_markers for rendering
     *
     * @param luhviz_markers
     */
    void createMarkerNs(const marker::LuhvizMarkers& luhviz_markers);

    /**
     * @brief filters the luhviz_markers, removes all markers from namespaces which are not selected
     *
     * @param luhviz_markers
     */
    void filterMarkers(marker::LuhvizMarkers& luhviz_markers) const;

   private:
    // ----- members -----
    const ImVec2 treeview_size = {320, 350};
    float checkbox_xoffset = treeview_size.x - 40;

    std::map<std::string, std::shared_ptr<MarkerNs>> marker_ns;

    std::map<std::string, bool> tree_expanded{};
    std::map<std::string, bool> checks{};
    std::deque<std::deque<bool>> item_checks{};

    luhsoccer::logger::Logger logger{"luhviz/inspector"};
    DataProxy& data_proxy;
    bool initialised{false};
    std::vector<std::string> disabled_ns;
    Fonts& fonts;
    // ----- methods -----

    /**
     * @brief renders a new Tree item with a checkbox
     *
     * @param text
     * @param i
     * @param checked
     * @param flag
     * @return true
     * @return false
     */
    bool addTreeItemCheckbox(const std::string& text, size_t i, bool& checked, int flag);

    /**
     * @brief renders a new checkbox
     *
     * @param text
     * @param i
     * @param checked
     */
    void addCheckbox(const std::string& text, size_t i, bool& checked);
};
}  // namespace luhsoccer::luhviz