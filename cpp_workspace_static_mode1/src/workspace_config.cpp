#include "workspace_config.h"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <fstream>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

namespace fs = std::filesystem;

namespace hcdr::workspace_static_mode1 {

namespace {

constexpr double kPi = 3.14159265358979323846;

double deg_to_rad(double degrees) {
    return degrees * kPi / 180.0;
}

std::string read_text_file(const fs::path& path) {
    std::ifstream input(path, std::ios::in | std::ios::binary);
    if (!input) {
        throw std::runtime_error("Failed to open config file: " + path.string());
    }

    std::ostringstream buffer;
    buffer << input.rdbuf();
    return buffer.str();
}

std::string trim_copy(const std::string& text) {
    const auto begin = std::find_if_not(text.begin(), text.end(), [](unsigned char c) {
        return std::isspace(c) != 0;
    });
    const auto end = std::find_if_not(text.rbegin(), text.rend(), [](unsigned char c) {
        return std::isspace(c) != 0;
    }).base();
    if (begin >= end) {
        return "";
    }
    return std::string(begin, end);
}

std::size_t skip_whitespace(const std::string& text, std::size_t start) {
    std::size_t index = start;
    while (index < text.size() && std::isspace(static_cast<unsigned char>(text.at(index))) != 0) {
        ++index;
    }
    return index;
}

std::string extract_json_value(const std::string& text, const std::string& key) {
    const std::string needle = "\"" + key + "\"";
    const std::size_t key_pos = text.find(needle);
    if (key_pos == std::string::npos) {
        return "";
    }

    const std::size_t colon_pos = text.find(':', key_pos + needle.size());
    if (colon_pos == std::string::npos) {
        throw std::runtime_error("Malformed JSON field: " + key);
    }

    std::size_t value_pos = skip_whitespace(text, colon_pos + 1);
    if (value_pos >= text.size()) {
        throw std::runtime_error("Missing JSON value for key: " + key);
    }

    const char first_char = text.at(value_pos);
    if (first_char == '"') {
        std::size_t cursor = value_pos + 1;
        bool escaped = false;
        while (cursor < text.size()) {
            const char current = text.at(cursor);
            if (current == '"' && !escaped) {
                return text.substr(value_pos, cursor - value_pos + 1);
            }
            escaped = (current == '\\' && !escaped);
            if (current != '\\') {
                escaped = false;
            }
            ++cursor;
        }
        throw std::runtime_error("Unterminated string value for key: " + key);
    }

    if (first_char == '{' || first_char == '[') {
        const char open_char = first_char;
        const char close_char = first_char == '{' ? '}' : ']';
        int depth = 0;
        bool inside_string = false;
        bool escaped = false;
        for (std::size_t cursor = value_pos; cursor < text.size(); ++cursor) {
            const char current = text.at(cursor);
            if (current == '"' && !escaped) {
                inside_string = !inside_string;
            } else if (!inside_string) {
                if (current == open_char) {
                    ++depth;
                } else if (current == close_char) {
                    --depth;
                    if (depth == 0) {
                        return text.substr(value_pos, cursor - value_pos + 1);
                    }
                }
            }

            escaped = (current == '\\' && !escaped);
            if (current != '\\') {
                escaped = false;
            }
        }
        throw std::runtime_error("Unterminated container value for key: " + key);
    }

    std::size_t end_pos = value_pos;
    while (end_pos < text.size() &&
           text.at(end_pos) != ',' &&
           text.at(end_pos) != '}' &&
           text.at(end_pos) != ']') {
        ++end_pos;
    }
    return trim_copy(text.substr(value_pos, end_pos - value_pos));
}

std::vector<std::string> split_top_level_csv(const std::string& text) {
    std::vector<std::string> parts;
    std::size_t token_start = 0;
    int brace_depth = 0;
    int bracket_depth = 0;
    bool inside_string = false;
    bool escaped = false;

    for (std::size_t index = 0; index < text.size(); ++index) {
        const char current = text.at(index);
        if (current == '"' && !escaped) {
            inside_string = !inside_string;
        } else if (!inside_string) {
            if (current == '{') {
                ++brace_depth;
            } else if (current == '}') {
                --brace_depth;
            } else if (current == '[') {
                ++bracket_depth;
            } else if (current == ']') {
                --bracket_depth;
            } else if (current == ',' && brace_depth == 0 && bracket_depth == 0) {
                parts.push_back(trim_copy(text.substr(token_start, index - token_start)));
                token_start = index + 1;
            }
        }

        escaped = (current == '\\' && !escaped);
        if (current != '\\') {
            escaped = false;
        }
    }

    if (token_start < text.size()) {
        parts.push_back(trim_copy(text.substr(token_start)));
    }
    return parts;
}

std::string strip_container(const std::string& text, char open_char, char close_char) {
    const std::string trimmed = trim_copy(text);
    if (trimmed.size() < 2U || trimmed.front() != open_char || trimmed.back() != close_char) {
        throw std::runtime_error("Unexpected JSON container: " + trimmed);
    }
    return trim_copy(trimmed.substr(1, trimmed.size() - 2U));
}

double parse_number(const std::string& text) {
    const std::string trimmed = trim_copy(text);
    if (trimmed.empty()) {
        throw std::runtime_error("Expected numeric JSON value.");
    }
    return std::stod(trimmed);
}

bool parse_bool(const std::string& text) {
    const std::string trimmed = trim_copy(text);
    if (trimmed == "true") {
        return true;
    }
    if (trimmed == "false") {
        return false;
    }
    throw std::runtime_error("Expected boolean JSON value.");
}

std::string parse_string(const std::string& text) {
    const std::string trimmed = trim_copy(text);
    if (trimmed.size() < 2U || trimmed.front() != '"' || trimmed.back() != '"') {
        throw std::runtime_error("Expected quoted JSON string.");
    }
    return trimmed.substr(1, trimmed.size() - 2U);
}

double get_number(const std::string& text, const std::string& key, double default_value) {
    const std::string value = extract_json_value(text, key);
    return value.empty() ? default_value : parse_number(value);
}

bool get_bool(const std::string& text, const std::string& key, bool default_value) {
    const std::string value = extract_json_value(text, key);
    return value.empty() ? default_value : parse_bool(value);
}

std::string get_string(const std::string& text, const std::string& key, const std::string& default_value) {
    const std::string value = extract_json_value(text, key);
    return value.empty() ? default_value : parse_string(value);
}

Eigen::Vector3d get_vector3(
    const std::string& text,
    const std::string& key,
    const Eigen::Vector3d& default_value) {
    const std::string value = extract_json_value(text, key);
    if (value.empty()) {
        return default_value;
    }

    const std::vector<std::string> entries = split_top_level_csv(strip_container(value, '[', ']'));
    if (entries.size() != 3U) {
        throw std::runtime_error("Expected three entries in array field: " + key);
    }

    Eigen::Vector3d output;
    for (std::size_t index = 0; index < 3U; ++index) {
        output(static_cast<Eigen::Index>(index)) = parse_number(entries.at(index));
    }
    return output;
}

Eigen::Matrix3d get_matrix3(
    const std::string& text,
    const std::string& key,
    const Eigen::Matrix3d& default_value) {
    const std::string value = extract_json_value(text, key);
    if (value.empty()) {
        return default_value;
    }

    const std::vector<std::string> row_strings = split_top_level_csv(strip_container(value, '[', ']'));
    if (row_strings.size() != 3U) {
        throw std::runtime_error("Expected three rows in matrix field: " + key);
    }

    Eigen::Matrix3d output = Eigen::Matrix3d::Zero();
    for (std::size_t row = 0; row < 3U; ++row) {
        const std::vector<std::string> entries =
            split_top_level_csv(strip_container(row_strings.at(row), '[', ']'));
        if (entries.size() != 3U) {
            throw std::runtime_error("Expected three columns in matrix field: " + key);
        }
        for (std::size_t col = 0; col < 3U; ++col) {
            output(static_cast<Eigen::Index>(row), static_cast<Eigen::Index>(col)) =
                parse_number(entries.at(col));
        }
    }
    return output;
}

Eigen::VectorXd get_vector_xd(
    const std::string& text,
    const std::string& key,
    const Eigen::VectorXd& default_value,
    std::size_t required_size) {
    const std::string value = extract_json_value(text, key);
    if (value.empty()) {
        return default_value;
    }

    const std::vector<std::string> entries = split_top_level_csv(strip_container(value, '[', ']'));
    if (entries.size() != required_size) {
        throw std::runtime_error("Unexpected array length in field: " + key);
    }

    Eigen::VectorXd output(static_cast<Eigen::Index>(required_size));
    for (std::size_t index = 0; index < required_size; ++index) {
        output(static_cast<Eigen::Index>(index)) = parse_number(entries.at(index));
    }
    return output;
}

std::vector<double> get_std_vector(
    const std::string& text,
    const std::string& key,
    const std::vector<double>& default_value) {
    const std::string value = extract_json_value(text, key);
    if (value.empty()) {
        return default_value;
    }

    const std::vector<std::string> entries = split_top_level_csv(strip_container(value, '[', ']'));
    std::vector<double> output;
    output.reserve(entries.size());
    for (const std::string& entry : entries) {
        output.push_back(parse_number(entry));
    }
    return output;
}

fs::path resolve_workspace_relative_path(const fs::path& workspace_root, const std::string& raw_path) {
    const fs::path candidate(raw_path);
    if (candidate.is_absolute()) {
        return candidate.lexically_normal();
    }
    return (workspace_root / candidate).lexically_normal();
}

}  // namespace

WorkspaceStaticConfig load_workspace_static_config(const fs::path& json_path) {
    WorkspaceStaticConfig config;
    config.config_path = fs::absolute(json_path);
    config.workspace_root = config.config_path.parent_path().parent_path();

    const std::string json_text = read_text_file(config.config_path);

    config.tension_min_n = get_number(json_text, "T_min", config.tension_min_n);
    config.tension_max_n = get_number(json_text, "T_max", config.tension_max_n);
    config.voxel_size_m = get_number(json_text, "voxel_size", config.voxel_size_m);
    config.platform_xy_resolution = static_cast<int>(std::llround(
        get_number(json_text, "mode1_xy_resolution", config.platform_xy_resolution)));
    config.psi_samples = static_cast<int>(std::llround(
        get_number(json_text, "psi_samples", config.psi_samples)));
    config.fixed_psi_rad = deg_to_rad(get_number(json_text, "fixed_psi_deg", 45.0));
    config.z0_m = get_number(json_text, "z0", config.z0_m);
    config.frame_half_length_m = get_number(json_text, "frameHalfLength", config.frame_half_length_m);
    config.frame_height_m = get_number(json_text, "frameHeight", config.frame_height_m);
    config.platform_half_side_m = get_number(json_text, "platform_half_side", config.platform_half_side_m);
    config.platform_half_thickness_m = get_number(
        json_text,
        "platform_half_thickness",
        config.platform_half_thickness_m);
    config.pulley_spacing_m = get_number(json_text, "pulley_spacing", config.pulley_spacing_m);
    const double default_scan_half_span = 0.95 * config.frame_half_length_m;
    config.platform_scan_half_span_m = get_number(
        json_text,
        "platform_xy_half_span",
        get_number(json_text, "platform_scan_half_span", default_scan_half_span));
    config.eps_rank = get_number(json_text, "eps_rank", config.eps_rank);
    config.gamma_feasible_tol_n = get_number(
        json_text,
        "gamma_feasible_tol_n",
        config.gamma_feasible_tol_n);
    config.output_png = get_bool(json_text, "output_png", config.output_png);
    config.output_bestpsi_csv = get_bool(json_text, "output_bestpsi_csv", config.output_bestpsi_csv);
    config.diagnostic_force_only_mode = get_bool(
        json_text,
        "diagnostic_force_only_mode",
        config.diagnostic_force_only_mode);
    config.force_only_torque_row_norm_eps = get_number(
        json_text,
        "force_only_torque_row_norm_eps",
        config.force_only_torque_row_norm_eps);
    config.gravity_mps2 = get_number(json_text, "gravity_mps2", config.gravity_mps2);
    config.qdot_norm = get_number(json_text, "qdot_norm", config.qdot_norm);
    config.qdd_norm = get_number(json_text, "qdd_norm", config.qdd_norm);

    const Eigen::VectorXd q_init_default = Eigen::VectorXd::Zero(6);
    config.q_m_init = get_vector_xd(json_text, "q_m_init", q_init_default, 6U);
    config.arm_base_offset_in_platform_m = get_vector3(
        json_text,
        "arm_base_offset_in_platform",
        config.arm_base_offset_in_platform_m);
    config.arm_base_rotation_in_platform = get_matrix3(
        json_text,
        "arm_base_rotation_in_platform",
        config.arm_base_rotation_in_platform);
    config.left_tip_body = get_string(json_text, "left_tip_body", config.left_tip_body);
    config.right_tip_body = get_string(json_text, "right_tip_body", config.right_tip_body);
    config.left_tip_local_m = get_vector3(json_text, "left_tip_local", config.left_tip_local_m);
    config.right_tip_local_m = get_vector3(json_text, "right_tip_local", config.right_tip_local_m);
    config.flange_body = get_string(json_text, "flange_body", config.flange_body);
    config.fixed_gripper_q = get_std_vector(json_text, "fixed_gripper_q", config.fixed_gripper_q);
    config.cable_routing_mode = get_string(json_text, "cable_routing_mode", config.cable_routing_mode);

    const std::vector<double> wrench_values = get_std_vector(json_text, "w_ext", {});
    if (!wrench_values.empty()) {
        if (wrench_values.size() != 6U) {
            throw std::runtime_error("w_ext must contain six numeric entries.");
        }
        for (std::size_t index = 0; index < 6U; ++index) {
            config.w_ext(static_cast<Eigen::Index>(index)) = wrench_values.at(index);
        }
    }

    config.urdf_path = resolve_workspace_relative_path(
        config.workspace_root,
        get_string(
            json_text,
            "urdf_path",
            "../kortex_description/robots/gen3_lite_gen3_lite_2f_local.urdf"));
    config.output_root = resolve_workspace_relative_path(
        config.workspace_root,
        get_string(json_text, "output_root", "output"));

    if (!fs::exists(config.urdf_path)) {
        throw std::runtime_error("URDF path does not exist: " + config.urdf_path.string());
    }
    if (config.platform_xy_resolution < 2 || config.psi_samples < 1) {
        throw std::runtime_error("platform_xy_resolution must be >= 2 and psi_samples >= 1.");
    }
    if (config.voxel_size_m <= 0.0) {
        throw std::runtime_error("voxel_size must be positive.");
    }
    if (config.tension_max_n <= config.tension_min_n) {
        throw std::runtime_error("T_max must be larger than T_min.");
    }
    if (config.platform_scan_half_span_m <= 0.0) {
        throw std::runtime_error("platform scan half span must be positive.");
    }
    if (config.eps_rank <= 0.0) {
        throw std::runtime_error("eps_rank must be positive.");
    }
    if (config.force_only_torque_row_norm_eps <= 0.0) {
        throw std::runtime_error("force_only_torque_row_norm_eps must be positive.");
    }

    return config;
}

}  // namespace hcdr::workspace_static_mode1
