// third_party/rerun_stub/rerun.hpp
#pragma once
#include <cstdint>
#include <initializer_list>
#include <string>
#include <vector>
#include <array>

namespace rerun {

struct Position2D {
    float x;
    float y;
};

struct Color {
    uint8_t r, g, b, a;
};

struct Arrows2D {
    // usato in action_selection.cpp:
    // Arrows2D::from_vectors({{..., ...}, {..., ...}})
    static Arrows2D from_vectors(std::initializer_list<std::array<float, 2>>) {
        return {};
    }

    // QUI: overload che accetta un float singolo  (with_radii(2.f))
    Arrows2D& with_radii(float) { return *this; }

    // E puoi tenere anche la versione con initializer_list, se vuoi
    Arrows2D& with_radii(std::initializer_list<float>) { return *this; }

    Arrows2D& with_origins(std::initializer_list<std::array<float, 2>>) {
        return *this;
    }

    Arrows2D& with_colors(std::initializer_list<std::array<uint8_t, 3>>) {
        return *this;
    }

    Arrows2D& with_labels(std::initializer_list<const char*>) {
        return *this;
    }

    Arrows2D& with_draw_order(float) { return *this; }
};

struct Points2D {
    explicit Points2D(const std::vector<Position2D>&) {}

    Points2D& with_radii(std::initializer_list<float>) { return *this; }

    Points2D& with_colors(const std::vector<Color>&) { return *this; }

    Points2D& with_draw_order(float) { return *this; }
};

struct RecordingStream {
    static RecordingStream& current() {
        static RecordingStream stream;
        return stream;
    }

    template <typename T>
    void log(const std::string&, const T&) {
        // stub: non fa nulla
    }
};

} // namespace rerun
