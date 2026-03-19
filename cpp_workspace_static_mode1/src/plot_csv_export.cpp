#include "plot_csv_export.h"

#include "voxel_grid.h"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cwchar>
#include <fstream>
#include <limits>
#include <stdexcept>
#include <vector>

#ifdef _WIN32
using std::max;
using std::min;

#include <windows.h>
#include <gdiplus.h>
#endif

namespace fs = std::filesystem;

namespace hcdr::workspace_static_mode1 {

namespace {

void ensure_parent_directory(const fs::path& output_path) {
    const fs::path parent_path = output_path.parent_path();
    if (!parent_path.empty()) {
        fs::create_directories(parent_path);
    }
}

bool compare_grid_records(const GridPointRecord& lhs, const GridPointRecord& rhs) {
    if (lhs.x_m != rhs.x_m) {
        return lhs.x_m < rhs.x_m;
    }
    return lhs.y_m < rhs.y_m;
}

bool compare_slice_summary(const PsiSliceSummary& lhs, const PsiSliceSummary& rhs) {
    return lhs.psi_rad < rhs.psi_rad;
}

#ifdef _WIN32
using Gdiplus::Bitmap;
using Gdiplus::Color;
using Gdiplus::Font;
using Gdiplus::FontFamily;
using Gdiplus::Graphics;
using Gdiplus::Pen;
using Gdiplus::SolidBrush;

class GdiplusSession {
public:
    GdiplusSession() {
        Gdiplus::GdiplusStartupInput input;
        if (Gdiplus::GdiplusStartup(&token_, &input, nullptr) != Gdiplus::Ok) {
            throw std::runtime_error("Failed to initialize GDI+.");
        }
    }

    ~GdiplusSession() {
        Gdiplus::GdiplusShutdown(token_);
    }

private:
    ULONG_PTR token_ = 0;
};

int get_png_encoder_clsid(CLSID* clsid) {
    UINT encoder_count = 0;
    UINT encoder_size = 0;
    Gdiplus::GetImageEncodersSize(&encoder_count, &encoder_size);
    if (encoder_size == 0) {
        return -1;
    }

    std::vector<std::uint8_t> buffer(encoder_size);
    auto* image_codec_info =
        reinterpret_cast<Gdiplus::ImageCodecInfo*>(buffer.data());
    Gdiplus::GetImageEncoders(encoder_count, encoder_size, image_codec_info);
    for (UINT index = 0; index < encoder_count; ++index) {
        if (std::wcscmp(image_codec_info[index].MimeType, L"image/png") == 0) {
            *clsid = image_codec_info[index].Clsid;
            return static_cast<int>(index);
        }
    }
    return -1;
}

Color color_from_gamma(double gamma_n, double min_gamma_n, double max_gamma_n, bool feasible) {
    if (!feasible || !std::isfinite(gamma_n)) {
        return Color(255, 220, 220, 220);
    }
    if (!(max_gamma_n > min_gamma_n)) {
        return Color(255, 30, 136, 229);
    }

    const double alpha = std::clamp((gamma_n - min_gamma_n) / (max_gamma_n - min_gamma_n), 0.0, 1.0);
    const int red = static_cast<int>(40.0 + 210.0 * alpha);
    const int green = static_cast<int>(90.0 + 110.0 * (1.0 - std::abs(alpha - 0.5) * 2.0));
    const int blue = static_cast<int>(220.0 - 180.0 * alpha);
    return Color(255, red, green, blue);
}

Color color_from_hit_count(std::uint32_t hit_count, double log_max_count) {
    const double value = std::log1p(static_cast<double>(hit_count));
    const double alpha = log_max_count > 0.0 ? std::clamp(value / log_max_count, 0.0, 1.0) : 0.0;
    const int red = static_cast<int>(20.0 + 200.0 * alpha);
    const int green = static_cast<int>(50.0 + 150.0 * alpha);
    const int blue = static_cast<int>(80.0 + 60.0 * (1.0 - alpha));
    return Color(255, red, green, blue);
}

void draw_axes_and_labels(
    Graphics& graphics,
    float plot_left,
    float plot_top,
    float plot_size,
    const std::wstring& title,
    const std::wstring& x_label,
    const std::wstring& y_label,
    double x_min,
    double x_max,
    double y_min,
    double y_max) {
    SolidBrush text_brush(Color(255, 20, 20, 20));
    Pen axis_pen(Color(255, 40, 40, 40), 2.0f);
    Pen grid_pen(Color(255, 225, 225, 225), 1.0f);

    graphics.DrawRectangle(&axis_pen, plot_left, plot_top, plot_size, plot_size);

    constexpr int tick_count = 10;
    for (int tick = 0; tick <= tick_count; ++tick) {
        const float coord = plot_left + plot_size * static_cast<float>(tick) / static_cast<float>(tick_count);
        graphics.DrawLine(&grid_pen, coord, plot_top, coord, plot_top + plot_size);
        graphics.DrawLine(&grid_pen, plot_left, coord, plot_left + plot_size, coord);
    }

    FontFamily font_family(L"Segoe UI");
    Font title_font(&font_family, 20.0f, Gdiplus::FontStyleBold, Gdiplus::UnitPixel);
    Font label_font(&font_family, 16.0f, Gdiplus::FontStyleRegular, Gdiplus::UnitPixel);
    Font tick_font(&font_family, 12.0f, Gdiplus::FontStyleRegular, Gdiplus::UnitPixel);

    graphics.DrawString(title.c_str(), -1, &title_font, Gdiplus::PointF(plot_left, 20.0f), &text_brush);
    graphics.DrawString(x_label.c_str(), -1, &label_font, Gdiplus::PointF(plot_left + plot_size * 0.35f, plot_top + plot_size + 40.0f), &text_brush);

    graphics.TranslateTransform(35.0f, plot_top + plot_size * 0.7f);
    graphics.RotateTransform(-90.0f);
    graphics.DrawString(y_label.c_str(), -1, &label_font, Gdiplus::PointF(0.0f, 0.0f), &text_brush);
    graphics.ResetTransform();

    const std::wstring x_min_text = std::to_wstring(x_min);
    const std::wstring x_max_text = std::to_wstring(x_max);
    const std::wstring y_min_text = std::to_wstring(y_min);
    const std::wstring y_max_text = std::to_wstring(y_max);
    graphics.DrawString(x_min_text.c_str(), -1, &tick_font, Gdiplus::PointF(plot_left - 10.0f, plot_top + plot_size + 10.0f), &text_brush);
    graphics.DrawString(x_max_text.c_str(), -1, &tick_font, Gdiplus::PointF(plot_left + plot_size - 55.0f, plot_top + plot_size + 10.0f), &text_brush);
    graphics.DrawString(y_min_text.c_str(), -1, &tick_font, Gdiplus::PointF(plot_left - 60.0f, plot_top + plot_size - 8.0f), &text_brush);
    graphics.DrawString(y_max_text.c_str(), -1, &tick_font, Gdiplus::PointF(plot_left - 60.0f, plot_top - 8.0f), &text_brush);
}

void save_bitmap_png(Bitmap& bitmap, const fs::path& output_path) {
    ensure_parent_directory(output_path);
    CLSID png_clsid{};
    if (get_png_encoder_clsid(&png_clsid) < 0) {
        throw std::runtime_error("PNG encoder is unavailable.");
    }
    if (bitmap.Save(output_path.wstring().c_str(), &png_clsid, nullptr) != Gdiplus::Ok) {
        throw std::runtime_error("Failed to save PNG: " + output_path.string());
    }
}

template <typename ColorFn>
void render_grid_png_common(
    const fs::path& output_path,
    const std::vector<GridPointRecord>& records,
    const std::wstring& title,
    ColorFn color_fn,
    bool draw_colorbar,
    double min_gamma_n,
    double max_gamma_n) {
    GdiplusSession session;
    constexpr int image_width = 1100;
    constexpr int image_height = 960;
    constexpr float plot_left = 120.0f;
    constexpr float plot_top = 90.0f;
    constexpr float plot_size = 760.0f;

    Bitmap bitmap(image_width, image_height, PixelFormat32bppARGB);
    Graphics graphics(&bitmap);
    graphics.Clear(Color(255, 255, 255, 255));
    graphics.SetSmoothingMode(Gdiplus::SmoothingModeNone);

    if (records.empty()) {
        save_bitmap_png(bitmap, output_path);
        return;
    }

    std::vector<double> unique_x;
    std::vector<double> unique_y;
    unique_x.reserve(records.size());
    unique_y.reserve(records.size());
    double x_min = std::numeric_limits<double>::infinity();
    double x_max = -std::numeric_limits<double>::infinity();
    double y_min = std::numeric_limits<double>::infinity();
    double y_max = -std::numeric_limits<double>::infinity();

    for (const GridPointRecord& record : records) {
        unique_x.push_back(record.x_m);
        unique_y.push_back(record.y_m);
        x_min = std::min(x_min, record.x_m);
        x_max = std::max(x_max, record.x_m);
        y_min = std::min(y_min, record.y_m);
        y_max = std::max(y_max, record.y_m);
    }

    std::sort(unique_x.begin(), unique_x.end());
    std::sort(unique_y.begin(), unique_y.end());
    unique_x.erase(std::unique(unique_x.begin(), unique_x.end()), unique_x.end());
    unique_y.erase(std::unique(unique_y.begin(), unique_y.end()), unique_y.end());

    const double step_x = unique_x.size() > 1U ? (unique_x.at(1) - unique_x.at(0)) : 1.0;
    const double step_y = unique_y.size() > 1U ? (unique_y.at(1) - unique_y.at(0)) : 1.0;
    const float cell_width = std::max(1.0f, plot_size / static_cast<float>(unique_x.size()));
    const float cell_height = std::max(1.0f, plot_size / static_cast<float>(unique_y.size()));

    for (const GridPointRecord& record : records) {
        const float pixel_x = plot_left +
            static_cast<float>((record.x_m - x_min + 0.5 * step_x) / (x_max - x_min + step_x) * plot_size);
        const float pixel_y = plot_top + plot_size -
            static_cast<float>((record.y_m - y_min + 0.5 * step_y) / (y_max - y_min + step_y) * plot_size);
        SolidBrush cell_brush(color_fn(record));
        graphics.FillRectangle(
            &cell_brush,
            pixel_x - 0.5f * cell_width,
            pixel_y - 0.5f * cell_height,
            cell_width,
            cell_height);
    }

    draw_axes_and_labels(
        graphics,
        plot_left,
        plot_top,
        plot_size,
        title,
        L"Platform X [m]",
        L"Platform Y [m]",
        x_min,
        x_max,
        y_min,
        y_max);

    if (draw_colorbar && std::isfinite(min_gamma_n) && std::isfinite(max_gamma_n) && max_gamma_n >= min_gamma_n) {
        const float bar_left = plot_left + plot_size + 40.0f;
        const float bar_top = plot_top + 40.0f;
        const float bar_height = plot_size - 80.0f;
        const float bar_width = 36.0f;
        for (int step = 0; step < 100; ++step) {
            const double alpha = static_cast<double>(step) / 99.0;
            const double gamma = min_gamma_n + alpha * (max_gamma_n - min_gamma_n);
            SolidBrush brush(color_from_gamma(gamma, min_gamma_n, max_gamma_n, true));
            const float y = bar_top + bar_height * static_cast<float>(1.0 - alpha);
            graphics.FillRectangle(&brush, bar_left, y, bar_width, bar_height / 100.0f + 1.0f);
        }

        FontFamily font_family(L"Segoe UI");
        Font label_font(&font_family, 12.0f, Gdiplus::FontStyleRegular, Gdiplus::UnitPixel);
        SolidBrush text_brush(Color(255, 20, 20, 20));
        const std::wstring max_text = std::to_wstring(max_gamma_n);
        const std::wstring min_text = std::to_wstring(min_gamma_n);
        graphics.DrawString(max_text.c_str(), -1, &label_font, Gdiplus::PointF(bar_left + 45.0f, bar_top - 8.0f), &text_brush);
        graphics.DrawString(min_text.c_str(), -1, &label_font, Gdiplus::PointF(bar_left + 45.0f, bar_top + bar_height - 8.0f), &text_brush);
        graphics.DrawString(L"Gamma [N]", -1, &label_font, Gdiplus::PointF(bar_left - 5.0f, bar_top + bar_height + 18.0f), &text_brush);
    }

    save_bitmap_png(bitmap, output_path);
}
#endif

}  // namespace

void write_fixedpsi_mask_csv(const fs::path& output_path, const std::vector<GridPointRecord>& records) {
    ensure_parent_directory(output_path);
    std::ofstream output(output_path);
    if (!output) {
        throw std::runtime_error("Failed to open fixed-psi mask CSV: " + output_path.string());
    }

    std::vector<GridPointRecord> ordered_records = records;
    std::sort(ordered_records.begin(), ordered_records.end(), compare_grid_records);

    output << "x_m,y_m,psi_rad,rank,sigma_min,rank_rejected,sigma_rejected,gamma_feasible,rejection_reason,solver_status\n";
    for (const GridPointRecord& record : ordered_records) {
        output << record.x_m << ','
               << record.y_m << ','
               << record.psi_rad << ','
               << record.rank << ','
               << record.sigma_min << ','
               << (record.rejection_reason == RejectionReason::RankDeficient ? 1 : 0) << ','
               << (record.rejection_reason == RejectionReason::SigmaTooSmall ? 1 : 0) << ','
               << (record.gamma_feasible ? 1 : 0) << ','
               << rejection_reason_name(record.rejection_reason) << ','
               << record.solver_status << '\n';
    }
}

void write_fixedpsi_gamma_csv(const fs::path& output_path, const std::vector<GridPointRecord>& records) {
    ensure_parent_directory(output_path);
    std::ofstream output(output_path);
    if (!output) {
        throw std::runtime_error("Failed to open fixed-psi gamma CSV: " + output_path.string());
    }

    std::vector<GridPointRecord> ordered_records = records;
    std::sort(ordered_records.begin(), ordered_records.end(), compare_grid_records);

    output << "x_m,y_m,psi_rad,gamma_n,gamma_feasible,rank,sigma_min,rejection_reason,solver_status\n";
    for (const GridPointRecord& record : ordered_records) {
        output << record.x_m << ','
               << record.y_m << ','
               << record.psi_rad << ','
               << record.gamma_n << ','
               << (record.gamma_feasible ? 1 : 0) << ','
               << record.rank << ','
               << record.sigma_min << ','
               << rejection_reason_name(record.rejection_reason) << ','
               << record.solver_status << '\n';
    }
}

void write_psi_slice_summary_csv(
    const fs::path& output_path,
    const std::vector<PsiSliceSummary>& summaries) {
    ensure_parent_directory(output_path);
    std::ofstream output(output_path);
    if (!output) {
        throw std::runtime_error("Failed to open psi slice summary CSV: " + output_path.string());
    }

    std::vector<PsiSliceSummary> ordered_summaries = summaries;
    std::sort(ordered_summaries.begin(), ordered_summaries.end(), compare_slice_summary);

    output << "psi_rad,psi_deg,scanned_grid_points,rank_rejected_points,sigma_rejected_points,gamma_feasible_points,min_gamma_n,max_gamma_n\n";
    for (const PsiSliceSummary& summary : ordered_summaries) {
        output << summary.psi_rad << ','
               << (summary.psi_rad * 180.0 / 3.14159265358979323846) << ','
               << summary.scanned_grid_points << ','
               << summary.rank_rejected_points << ','
               << summary.sigma_rejected_points << ','
               << summary.gamma_feasible_points << ','
               << summary.min_gamma_n << ','
               << summary.max_gamma_n << '\n';
    }
}

void write_log_file(const fs::path& output_path, const std::vector<std::string>& log_lines) {
    ensure_parent_directory(output_path);
    std::ofstream output(output_path);
    if (!output) {
        throw std::runtime_error("Failed to open log file: " + output_path.string());
    }

    for (const std::string& log_line : log_lines) {
        output << log_line << '\n';
    }
}

void write_output_file_list(const fs::path& output_path, const std::vector<fs::path>& files) {
    ensure_parent_directory(output_path);
    std::ofstream output(output_path);
    if (!output) {
        throw std::runtime_error("Failed to open output file list: " + output_path.string());
    }

    std::vector<fs::path> ordered_files = files;
    std::sort(ordered_files.begin(), ordered_files.end());
    for (const fs::path& file_path : ordered_files) {
        output << file_path.filename().string() << '\n';
    }
}

void render_fixedpsi_mask_png(
    const fs::path& output_path,
    const std::vector<GridPointRecord>& records,
    double fixed_psi_rad) {
#ifdef _WIN32
    const std::wstring title =
        L"Mode 1 Platform Center Fixed Psi = " +
        std::to_wstring(static_cast<int>(std::llround(fixed_psi_rad * 180.0 / 3.14159265358979323846))) +
        L" deg";
    render_grid_png_common(
        output_path,
        records,
        title,
        [](const GridPointRecord& record) {
            return record.gamma_feasible ? Color(255, 28, 93, 153) : Color(255, 238, 238, 238);
        },
        false,
        std::numeric_limits<double>::quiet_NaN(),
        std::numeric_limits<double>::quiet_NaN());
#else
    (void)output_path;
    (void)records;
    (void)fixed_psi_rad;
#endif
}

void render_fixedpsi_gamma_png(
    const fs::path& output_path,
    const std::vector<GridPointRecord>& records,
    double fixed_psi_rad) {
#ifdef _WIN32
    double min_gamma_n = std::numeric_limits<double>::infinity();
    double max_gamma_n = -std::numeric_limits<double>::infinity();
    for (const GridPointRecord& record : records) {
        if (record.gamma_feasible && std::isfinite(record.gamma_n)) {
            min_gamma_n = std::min(min_gamma_n, record.gamma_n);
            max_gamma_n = std::max(max_gamma_n, record.gamma_n);
        }
    }
    if (!std::isfinite(min_gamma_n)) {
        min_gamma_n = std::numeric_limits<double>::quiet_NaN();
        max_gamma_n = std::numeric_limits<double>::quiet_NaN();
    }

    const std::wstring title =
        L"Mode 1 Platform Center Gamma, Fixed Psi = " +
        std::to_wstring(static_cast<int>(std::llround(fixed_psi_rad * 180.0 / 3.14159265358979323846))) +
        L" deg";
    render_grid_png_common(
        output_path,
        records,
        title,
        [min_gamma_n, max_gamma_n](const GridPointRecord& record) {
            return color_from_gamma(record.gamma_n, min_gamma_n, max_gamma_n, record.gamma_feasible);
        },
        true,
        min_gamma_n,
        max_gamma_n);
#else
    (void)output_path;
    (void)records;
    (void)fixed_psi_rad;
#endif
}

void render_projection_png(
    const fs::path& output_path,
    const std::vector<PointRecord>& points,
    const std::string& title,
    const std::string& x_label,
    const std::string& y_label,
    double voxel_size_m) {
#ifdef _WIN32
    GdiplusSession session;
    constexpr int image_width = 1100;
    constexpr int image_height = 960;
    constexpr float plot_left = 120.0f;
    constexpr float plot_top = 90.0f;
    constexpr float plot_size = 760.0f;

    Bitmap bitmap(image_width, image_height, PixelFormat32bppARGB);
    Graphics graphics(&bitmap);
    graphics.Clear(Color(255, 255, 255, 255));
    graphics.SetSmoothingMode(Gdiplus::SmoothingModeNone);

    const std::vector<ProjectionRecord> projections =
        build_projection_records(points, voxel_size_m, 0, 1);
    if (!projections.empty()) {
        double u_min = std::numeric_limits<double>::infinity();
        double u_max = -std::numeric_limits<double>::infinity();
        double v_min = std::numeric_limits<double>::infinity();
        double v_max = -std::numeric_limits<double>::infinity();
        double log_max_count = 0.0;
        for (const ProjectionRecord& projection : projections) {
            u_min = std::min(u_min, projection.axis_u_m);
            u_max = std::max(u_max, projection.axis_u_m);
            v_min = std::min(v_min, projection.axis_v_m);
            v_max = std::max(v_max, projection.axis_v_m);
            log_max_count = std::max(log_max_count, std::log1p(static_cast<double>(projection.hit_count)));
        }

        for (const ProjectionRecord& projection : projections) {
            const float pixel_x = plot_left +
                static_cast<float>((projection.axis_u_m - u_min + 0.5 * voxel_size_m) / (u_max - u_min + voxel_size_m) * plot_size);
            const float pixel_y = plot_top + plot_size -
                static_cast<float>((projection.axis_v_m - v_min + 0.5 * voxel_size_m) / (v_max - v_min + voxel_size_m) * plot_size);
            SolidBrush brush(color_from_hit_count(projection.hit_count, log_max_count));
            graphics.FillRectangle(&brush, pixel_x - 2.0f, pixel_y - 2.0f, 4.0f, 4.0f);
        }

        draw_axes_and_labels(
            graphics,
            plot_left,
            plot_top,
            plot_size,
            std::wstring(title.begin(), title.end()),
            std::wstring(x_label.begin(), x_label.end()),
            std::wstring(y_label.begin(), y_label.end()),
            u_min,
            u_max,
            v_min,
            v_max);
    }

    save_bitmap_png(bitmap, output_path);
#else
    (void)output_path;
    (void)points;
    (void)title;
    (void)x_label;
    (void)y_label;
    (void)voxel_size_m;
#endif
}

}  // namespace hcdr::workspace_static_mode1
