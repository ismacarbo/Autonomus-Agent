#include "mvc/model/reporting/slam_reference_exporter.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <fstream>
#include <vector>

namespace thesis_sim::mvc::model {
namespace {

struct Color {
    std::uint8_t r = 0;
    std::uint8_t g = 0;
    std::uint8_t b = 0;
    std::uint8_t a = 255;
};

struct Raster {
    int width = 0;
    int height = 0;
    std::vector<std::uint8_t> rgba;
};

struct Transform {
    Rect bounds;
    int x = 0;
    int y = 0;
    int width = 1;
    int height = 1;
    double scale = 1.0;
};

void append_be32(std::vector<std::uint8_t>* out, std::uint32_t value) {
    out->push_back(static_cast<std::uint8_t>((value >> 24) & 0xff));
    out->push_back(static_cast<std::uint8_t>((value >> 16) & 0xff));
    out->push_back(static_cast<std::uint8_t>((value >> 8) & 0xff));
    out->push_back(static_cast<std::uint8_t>(value & 0xff));
}

std::uint32_t crc_update(std::uint32_t crc, std::uint8_t byte) {
    crc ^= byte;
    for (int bit = 0; bit < 8; ++bit) {
        crc = (crc & 1u) ? (0xedb88320u ^ (crc >> 1)) : (crc >> 1);
    }
    return crc;
}

std::uint32_t crc32(const char type[4], const std::vector<std::uint8_t>& data) {
    std::uint32_t crc = 0xffffffffu;
    for (int i = 0; i < 4; ++i) {
        crc = crc_update(crc, static_cast<std::uint8_t>(type[i]));
    }
    for (std::uint8_t byte : data) {
        crc = crc_update(crc, byte);
    }
    return crc ^ 0xffffffffu;
}

std::uint32_t adler32(const std::vector<std::uint8_t>& data) {
    constexpr std::uint32_t kMod = 65521u;
    std::uint32_t a = 1u;
    std::uint32_t b = 0u;
    for (std::uint8_t byte : data) {
        a = (a + byte) % kMod;
        b = (b + a) % kMod;
    }
    return (b << 16) | a;
}

void write_chunk(std::ofstream& out,
                 const char type[4],
                 const std::vector<std::uint8_t>& data) {
    std::vector<std::uint8_t> bytes;
    append_be32(&bytes, static_cast<std::uint32_t>(data.size()));
    out.write(reinterpret_cast<const char*>(bytes.data()), bytes.size());
    out.write(type, 4);
    if (!data.empty()) {
        out.write(reinterpret_cast<const char*>(data.data()), data.size());
    }
    bytes.clear();
    append_be32(&bytes, crc32(type, data));
    out.write(reinterpret_cast<const char*>(bytes.data()), bytes.size());
}

std::vector<std::uint8_t> make_stored_zlib(const std::vector<std::uint8_t>& raw) {
    std::vector<std::uint8_t> zlib{0x78, 0x01};
    std::size_t offset = 0;
    while (offset < raw.size() || (raw.empty() && offset == 0)) {
        const std::uint16_t length = static_cast<std::uint16_t>(
            std::min<std::size_t>(raw.size() - offset, 65535));
        const bool final = offset + length >= raw.size();
        zlib.push_back(final ? 0x01 : 0x00);
        zlib.push_back(static_cast<std::uint8_t>(length & 0xff));
        zlib.push_back(static_cast<std::uint8_t>((length >> 8) & 0xff));
        const std::uint16_t inverse = static_cast<std::uint16_t>(~length);
        zlib.push_back(static_cast<std::uint8_t>(inverse & 0xff));
        zlib.push_back(static_cast<std::uint8_t>((inverse >> 8) & 0xff));
        zlib.insert(zlib.end(), raw.begin() + static_cast<std::ptrdiff_t>(offset),
                    raw.begin() + static_cast<std::ptrdiff_t>(offset + length));
        offset += length;
        if (raw.empty()) {
            break;
        }
    }
    append_be32(&zlib, adler32(raw));
    return zlib;
}

bool write_png(const std::string& path, const Raster& raster) {
    std::ofstream out(path, std::ios::out | std::ios::binary | std::ios::trunc);
    if (!out.is_open() || raster.width <= 0 || raster.height <= 0) {
        return false;
    }
    const std::array<std::uint8_t, 8> signature{{137, 80, 78, 71, 13, 10, 26, 10}};
    out.write(reinterpret_cast<const char*>(signature.data()), signature.size());
    std::vector<std::uint8_t> header;
    append_be32(&header, static_cast<std::uint32_t>(raster.width));
    append_be32(&header, static_cast<std::uint32_t>(raster.height));
    header.insert(header.end(), {8, 6, 0, 0, 0});
    write_chunk(out, "IHDR", header);
    std::vector<std::uint8_t> raw;
    raw.reserve(static_cast<std::size_t>((raster.width * 4 + 1) * raster.height));
    for (int y = 0; y < raster.height; ++y) {
        raw.push_back(0);
        const auto begin = raster.rgba.begin() + static_cast<std::ptrdiff_t>(y * raster.width * 4);
        raw.insert(raw.end(), begin, begin + raster.width * 4);
    }
    write_chunk(out, "IDAT", make_stored_zlib(raw));
    write_chunk(out, "IEND", {});
    return out.good();
}

Raster make_raster(int width, int height, Color fill) {
    Raster raster{width, height, std::vector<std::uint8_t>(
        static_cast<std::size_t>(width * height * 4))};
    for (std::size_t i = 0; i < raster.rgba.size(); i += 4) {
        raster.rgba[i] = fill.r;
        raster.rgba[i + 1] = fill.g;
        raster.rgba[i + 2] = fill.b;
        raster.rgba[i + 3] = fill.a;
    }
    return raster;
}

void set_pixel(Raster* raster, int x, int y, Color color) {
    if (x < 0 || y < 0 || x >= raster->width || y >= raster->height) {
        return;
    }
    const std::size_t offset = static_cast<std::size_t>((y * raster->width + x) * 4);
    raster->rgba[offset] = color.r;
    raster->rgba[offset + 1] = color.g;
    raster->rgba[offset + 2] = color.b;
    raster->rgba[offset + 3] = color.a;
}

void draw_line(Raster* raster, int x0, int y0, int x1, int y1, Color color) {
    const int dx = std::abs(x1 - x0);
    const int sx = x0 < x1 ? 1 : -1;
    const int dy = -std::abs(y1 - y0);
    const int sy = y0 < y1 ? 1 : -1;
    int error = dx + dy;
    for (;;) {
        set_pixel(raster, x0, y0, color);
        if (x0 == x1 && y0 == y1) {
            break;
        }
        const int doubled = 2 * error;
        if (doubled >= dy) {
            error += dy;
            x0 += sx;
        }
        if (doubled <= dx) {
            error += dx;
            y0 += sy;
        }
    }
}

void draw_disk(Raster* raster, int cx, int cy, int radius, Color color) {
    for (int y = -radius; y <= radius; ++y) {
        for (int x = -radius; x <= radius; ++x) {
            if (x * x + y * y <= radius * radius) {
                set_pixel(raster, cx + x, cy + y, color);
            }
        }
    }
}

Transform make_transform(Rect bounds, int width, int height) {
    constexpr int kLeft = 64;
    constexpr int kTop = 42;
    constexpr int kRight = 32;
    constexpr int kBottom = 52;
    const double initial_x = std::max(bounds.max_x - bounds.min_x, 0.2);
    const double initial_y = std::max(bounds.max_y - bounds.min_y, 0.2);
    const double padding = std::max(0.08, 0.08 * std::max(initial_x, initial_y));
    bounds = {bounds.min_x - padding, bounds.min_y - padding,
              bounds.max_x + padding, bounds.max_y + padding};
    const int plot_width = std::max(width - kLeft - kRight, 64);
    const int plot_height = std::max(height - kTop - kBottom, 64);
    double span_x = bounds.max_x - bounds.min_x;
    double span_y = bounds.max_y - bounds.min_y;
    const double plot_aspect = static_cast<double>(plot_width) / plot_height;
    if (span_x / span_y > plot_aspect) {
        const double desired = span_x / plot_aspect;
        const double center = (bounds.min_y + bounds.max_y) * 0.5;
        bounds.min_y = center - desired * 0.5;
        bounds.max_y = center + desired * 0.5;
        span_y = desired;
    } else {
        const double desired = span_y * plot_aspect;
        const double center = (bounds.min_x + bounds.max_x) * 0.5;
        bounds.min_x = center - desired * 0.5;
        bounds.max_x = center + desired * 0.5;
        span_x = desired;
    }
    return {bounds, kLeft, kTop, plot_width, plot_height,
            std::min(plot_width / span_x, plot_height / span_y)};
}

std::pair<int, int> to_image(const Transform& transform, const Vec2& point) {
    return {
        static_cast<int>(std::lround(transform.x +
            (point.x - transform.bounds.min_x) * transform.scale)),
        static_cast<int>(std::lround(transform.y + transform.height -
            (point.y - transform.bounds.min_y) * transform.scale)),
    };
}

void draw_world_line(Raster* raster, const Transform& transform,
                     const Vec2& a, const Vec2& b, Color color, int radius = 0) {
    const auto [x0, y0] = to_image(transform, a);
    const auto [x1, y1] = to_image(transform, b);
    for (int oy = -radius; oy <= radius; ++oy) {
        for (int ox = -radius; ox <= radius; ++ox) {
            if (ox * ox + oy * oy <= radius * radius) {
                draw_line(raster, x0 + ox, y0 + oy, x1 + ox, y1 + oy, color);
            }
        }
    }
}

void draw_world_disk(Raster* raster, const Transform& transform,
                     const Vec2& point, int radius, Color color) {
    const auto [x, y] = to_image(transform, point);
    draw_disk(raster, x, y, radius, color);
}

void draw_world_rect(Raster* raster, const Transform& transform,
                     const Rect& rect, Color color) {
    const auto [ax, ay] = to_image(transform, {rect.min_x, rect.min_y});
    const auto [bx, by] = to_image(transform, {rect.max_x, rect.max_y});
    for (int y = std::max(0, std::min(ay, by));
         y <= std::min(raster->height - 1, std::max(ay, by)); ++y) {
        for (int x = std::max(0, std::min(ax, bx));
             x <= std::min(raster->width - 1, std::max(ax, bx)); ++x) {
            set_pixel(raster, x, y, color);
        }
    }
}

double grid_step(double target) {
    const double base = std::pow(10.0, std::floor(std::log10(std::max(target, 1e-6))));
    const double fraction = target / base;
    return (fraction <= 1.0 ? 1.0 : (fraction <= 2.0 ? 2.0 : (fraction <= 5.0 ? 5.0 : 10.0))) * base;
}

void draw_grid(Raster* raster, const Transform& transform) {
    const double step = grid_step(std::max(
        transform.bounds.max_x - transform.bounds.min_x,
        transform.bounds.max_y - transform.bounds.min_y) / 8.0);
    const Color color{126, 126, 126, 255};
    for (double x = std::floor(transform.bounds.min_x / step) * step;
         x <= transform.bounds.max_x + 1e-9; x += step) {
        draw_world_line(raster, transform,
                        {x, transform.bounds.min_y}, {x, transform.bounds.max_y}, color);
    }
    for (double y = std::floor(transform.bounds.min_y / step) * step;
         y <= transform.bounds.max_y + 1e-9; y += step) {
        draw_world_line(raster, transform,
                        {transform.bounds.min_x, y}, {transform.bounds.max_x, y}, color);
    }
}

}  // namespace

bool write_slam_reference_png(const SlamReferenceArtifact& artifact,
                              const std::string& path) {
    constexpr int kWidth = 1200;
    constexpr int kHeight = 900;
    Raster raster = make_raster(kWidth, kHeight, {246, 246, 246, 255});
    const Transform transform = make_transform(artifact.bounds, kWidth, kHeight);
    for (int y = transform.y; y <= transform.y + transform.height; ++y) {
        for (int x = transform.x; x <= transform.x + transform.width; ++x) {
            set_pixel(&raster, x, y, {142, 142, 142, 255});
        }
    }

    const std::size_t ray_stride = artifact.free_space_rays.size() > 220000
        ? artifact.free_space_rays.size() / 220000 + 1
        : 1;
    for (std::size_t i = 0; i < artifact.free_space_rays.size(); i += ray_stride) {
        draw_world_line(&raster, transform,
                        artifact.free_space_rays[i].first,
                        artifact.free_space_rays[i].second,
                        {246, 246, 246, 255});
    }
    draw_grid(&raster, transform);

    if (artifact.draw_reference_geometry) {
        for (const Rect& obstacle : artifact.reference_obstacles) {
            draw_world_rect(&raster, transform, obstacle, {88, 94, 101, 255});
        }
        for (std::size_t i = 1; i < artifact.reference_road.size(); ++i) {
            draw_world_line(&raster, transform,
                            artifact.reference_road[i - 1], artifact.reference_road[i],
                            {42, 125, 102, 255});
        }
    }
    const int occupied_radius = transform.scale > 420.0 ? 2 : 1;
    for (const Vec2& point : artifact.occupied_points) {
        draw_world_disk(&raster, transform, point, occupied_radius, {18, 18, 18, 255});
    }
    for (std::size_t i = 1; i < artifact.estimated_trail.size(); ++i) {
        draw_world_line(&raster, transform,
                        artifact.estimated_trail[i - 1], artifact.estimated_trail[i],
                        {20, 92, 230, 255}, 1);
    }
    if (artifact.draw_mission_markers) {
        draw_world_disk(&raster, transform, artifact.start, 7, {42, 170, 80, 255});
        draw_world_disk(&raster, transform, artifact.goal, 5, {220, 56, 48, 255});
        draw_world_disk(&raster, transform, artifact.current, 5, {244, 128, 36, 255});
    } else if (!artifact.estimated_trail.empty()) {
        draw_world_disk(&raster, transform, artifact.estimated_trail.front(), 6, {42, 170, 80, 255});
        draw_world_disk(&raster, transform, artifact.estimated_trail.back(), 6, {220, 56, 48, 255});
    }
    return write_png(path, raster);
}

}  // namespace thesis_sim::mvc::model
