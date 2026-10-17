// Shared presentation primitives. Navigation and control remain in their own modules.
const ImVec4 kUiAccent(0.37f, 0.84f, 0.75f, 1.0f);
const ImVec4 kUiMuted(0.58f, 0.66f, 0.74f, 1.0f);
const ImVec4 kUiWarning(0.98f, 0.73f, 0.38f, 1.0f);
const ImVec4 kUiDanger(0.98f, 0.43f, 0.43f, 1.0f);
ImFont* ui_title_font = nullptr;
ImFont* ui_value_font = nullptr;

void configure_presentation_fonts() {
    ImGuiIO& io = ImGui::GetIO();
    const char* font_paths[] = {
        "/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf",
        "/usr/share/fonts/dejavu/DejaVuSans.ttf",
        "/usr/share/fonts/dejavu-sans-fonts/DejaVuSans.ttf",
        "/usr/share/fonts/liberation-fonts/LiberationSans-Regular.ttf",
    };
    const char* selected = nullptr;
    for (const char* path : font_paths) {
        if (std::filesystem::is_regular_file(path)) { selected = path; break; }
    }
    auto load = [&](float size) {
        ImFontConfig config;
        config.SizePixels = size;
        return selected ? io.Fonts->AddFontFromFileTTF(selected, size, &config)
                        : io.Fonts->AddFontDefault(&config);
    };
    io.FontDefault = load(16.0f);
    ui_title_font = load(22.0f);
    ui_value_font = load(25.0f);
    ImPlot::StyleColorsDark();
    ImPlotStyle& plot = ImPlot::GetStyle();
    plot.FitPadding = ImVec2(0.05f, 0.12f);
    plot.PlotPadding = ImVec2(14, 12);
    plot.LegendPadding = ImVec2(8, 8);
    plot.Colors[ImPlotCol_FrameBg] = ImVec4(0.07f, 0.09f, 0.12f, 1);
    plot.Colors[ImPlotCol_PlotBg] = ImVec4(0.055f, 0.075f, 0.10f, 1);
    plot.Colors[ImPlotCol_PlotBorder] = ImVec4(0.16f, 0.20f, 0.25f, 1);
}

const char* short_scenario_name(EnvironmentMode mode) {
    switch (mode) {
        case EnvironmentMode::StructuredRoad: return "Structured";
        case EnvironmentMode::MixedRoadGates: return "Mixed";
        default: return "Unstructured";
    }
}

void ui_badge(const char* text, const ImVec4& color) {
    const ImVec2 p = ImGui::GetCursorScreenPos();
    const ImVec2 size(ImGui::CalcTextSize(text).x + 28, ImGui::GetTextLineHeight() + 12);
    ImDrawList* draw = ImGui::GetWindowDrawList();
    draw->AddRectFilled(p, ImVec2(p.x + size.x, p.y + size.y),
                       ImGui::ColorConvertFloat4ToU32(ImVec4(color.x, color.y, color.z, 0.12f)), 5);
    draw->AddCircleFilled(ImVec2(p.x + 10, p.y + size.y * 0.5f), 3,
                          ImGui::ColorConvertFloat4ToU32(color));
    draw->AddText(ImVec2(p.x + 19, p.y + 6), ImGui::ColorConvertFloat4ToU32(color), text);
    ImGui::Dummy(size);
}

bool ui_choice(const char* label, bool selected, float width = 0) {
    ImGui::PushStyleColor(ImGuiCol_Button, selected ? ImVec4(0.12f, 0.29f, 0.29f, 1) : ImVec4(0.09f, 0.12f, 0.16f, 1));
    ImGui::PushStyleColor(ImGuiCol_Text, selected ? kUiAccent : kUiMuted);
    const bool clicked = ImGui::Button(label, ImVec2(width, 34));
    ImGui::PopStyleColor(2);
    return clicked;
}

void ui_metric(const char* id, const char* title, const std::string& value,
               const char* detail, const ImVec4& color = kUiAccent) {
    ImGui::PushID(id);
    if (ImGui::BeginChild("metric", ImVec2(0, 98), true, ImGuiWindowFlags_NoScrollbar)) {
        ImGui::TextDisabled("%s", title);
        ImGui::PushFont(ui_value_font);
        ImGui::TextColored(color, "%s", value.c_str());
        ImGui::PopFont();
        ImGui::TextDisabled("%s", detail);
    }
    ImGui::EndChild();
    ImGui::PopID();
}

std::string ui_number(double value, const char* unit, int decimals = 2) {
    if (!std::isfinite(value)) return "--";
    char text[64];
    std::snprintf(text, sizeof(text), "%.*f%s", decimals, value, unit);
    return text;
}

void ui_detail(const char* label, const std::string& value, const ImVec4& color = kUiMuted) {
    ImGui::PushID(label);
    if (ImGui::BeginTable("detail", 2, ImGuiTableFlags_SizingStretchProp)) {
        ImGui::TableSetupColumn("label", ImGuiTableColumnFlags_WidthStretch, 0.9f);
        ImGui::TableSetupColumn("value", ImGuiTableColumnFlags_WidthStretch, 1.1f);
        ImGui::TableNextColumn();
        ImGui::TextDisabled("%s", label);
        ImGui::TableNextColumn();
        ImGui::PushStyleColor(ImGuiCol_Text, color);
        ImGui::TextWrapped("%s", value.c_str());
        ImGui::PopStyleColor();
        ImGui::EndTable();
    }
    ImGui::PopID();
}

void render_map_toolbar(UiState* state, const WorldMap& world, bool live) {
    ImGui::AlignTextToFramePadding();
    ImGui::TextUnformatted(state->map_editor_enabled ? "Map editor" : "Navigation map");
    ImGui::SameLine();
    ImGui::TextDisabled("/ %s", short_scenario_name(world.environment_mode()));
    const float right = ImGui::GetWindowContentRegionMax().x;
    if (ImGui::GetWindowWidth() >= 630) ImGui::SameLine(std::max(ImGui::GetCursorPosX(), right - 218.0f));
    if (ui_choice("SLAM", state->show_slam_map, 88)) state->show_slam_map = !state->show_slam_map;
    ImGui::SameLine();
    if (ImGui::Button("Layers", ImVec2(110, 34))) ImGui::OpenPopup("MapLayers");
    if (ImGui::BeginPopup("MapLayers")) {
        ImGui::TextDisabled("MAP LAYERS");
        ImGui::Separator();
        ImGui::Checkbox("Robot footprint (true scale)", &state->show_robot_footprint);
        ImGui::Checkbox("Metric grid", &state->show_grid);
        ImGui::Checkbox("Trails & MPC reference", &state->show_trails);
        ImGui::Checkbox("LiDAR endpoints", &state->show_lidar_hits);
        ImGui::Checkbox("LiDAR rays", &state->show_lidar_rays);
        ImGui::Checkbox("Gate labels", &state->show_gate_labels);
        ImGui::Checkbox("Detailed overlays", &state->show_world_hud);
        ImGui::EndPopup();
    }
    ImGui::TextDisabled(live ? "Recorded pose" : "Scene preview");
    ImGui::SameLine();
    ImGui::TextColored(ImGui::ColorConvertU32ToFloat4(kColorEstimateTrail), "  Pose / trail");
    ImGui::SameLine();
    ImGui::TextColored(ImGui::ColorConvertU32ToFloat4(kColorTrajectory), "  MPC path");
    if (ImGui::GetWindowWidth() >= 480) {
        ImGui::SameLine();
        ImGui::TextColored(ImGui::ColorConvertU32ToFloat4(kColorGate), "  Gates");
    }
}

struct MapViewportLayout {
    ImVec2 map_pos, map_size, slam_pos, slam_size;
};
MapViewportLayout map_viewport_layout(ImVec2 pos, ImVec2 size, bool split) {
    MapViewportLayout layout{pos, size, pos, ImVec2(0, 0)};
    if (split) {
        if (size.x >= 760) {
            layout.map_size.x = (size.x - 12) * 0.53f;
            layout.slam_pos.x += layout.map_size.x + 12;
            layout.slam_size = ImVec2(size.x - layout.map_size.x - 12, size.y);
        } else {
            layout.map_size.y = (size.y - 12) * 0.5f;
            layout.slam_pos.y += layout.map_size.y + 12;
            layout.slam_size = ImVec2(size.x, size.y - layout.map_size.y - 12);
        }
    }
    return layout;
}

void draw_metric_map_caption(ImDrawList* draw, const CanvasTransform& tx,
                             const WorldMap& world, ImVec2 pos, ImVec2 size) {
    char caption[96];
    const Rect bounds = world.bounds();
    std::snprintf(caption, sizeof(caption), "%.2f x %.2f m   /   x right, y up",
                  bounds.max_x - bounds.min_x, bounds.max_y - bounds.min_y);
    draw->AddText(ImVec2(pos.x + 12, pos.y + size.y - 22), IM_COL32(146, 166, 181, 255), caption);
    const ImVec2 start = world_to_screen(tx, world.start());
    const Vec2 tip{world.start().x + 0.10 * std::cos(world.start_heading()),
                   world.start().y + 0.10 * std::sin(world.start_heading())};
    draw->AddCircle(start, 5, IM_COL32(95, 215, 191, 220), 0, 1.5f);
    draw->AddLine(start, world_to_screen(tx, tip), IM_COL32(95, 215, 191, 220), 2);
    draw->AddText(ImVec2(start.x - 43, start.y + 7), IM_COL32(95, 215, 191, 255), "start");
}

void draw_robot_footprint(ImDrawList* draw, const CanvasTransform& tx, Vec2 position,
                          double yaw, double length, double width) {
    ImVec2 corners[4];
    const double x[] = {-length * 0.5, length * 0.5, length * 0.5, -length * 0.5};
    const double y[] = {-width * 0.5, -width * 0.5, width * 0.5, width * 0.5};
    for (int i = 0; i < 4; ++i)
        corners[i] = world_to_screen(tx, {position.x + std::cos(yaw) * x[i] - std::sin(yaw) * y[i],
                                          position.y + std::sin(yaw) * x[i] + std::cos(yaw) * y[i]});
    draw->AddConvexPolyFilled(corners, 4, IM_COL32(155, 195, 221, 18));
    draw->AddPolyline(corners, 4, IM_COL32(155, 195, 221, 170), ImDrawFlags_Closed, 1.3f);
}

bool ui_labeled_combo(const char* label, int* selected, const char* const items[], int count) {
    ImGui::TextDisabled("%s", label);
    ImGui::SetNextItemWidth(-1);
    const std::string id = std::string("##") + label;
    return ImGui::Combo(id.c_str(), selected, items, count);
}
