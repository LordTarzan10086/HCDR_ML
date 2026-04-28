function out = gen3_lite_paper_tables_figure(summary, varargin)
%GEN3_LITE_PAPER_TABLES_FIGURE Render paper-style DH/limit tables as a PNG.
%
%   OUT = GEN3_LITE_PAPER_TABLES_FIGURE(SUMMARY) draws two compact tables:
%   the official classical DH table and the official joint-angle ranges.
%
%   OUT = GEN3_LITE_PAPER_TABLES_FIGURE(..., "SavePath", PATH) exports the
%   figure to PATH using exportgraphics.

    if ~isstruct(summary) || ~isscalar(summary)
        error("HCDR:ArgInvalid", "summary must be a scalar struct.");
    end

    parser = inputParser;
    parser.addParameter("Visible", false, @(x) islogical(x) || isnumeric(x));
    parser.addParameter("SavePath", "", @(x) ischar(x) || isstring(x));
    parser.parse(varargin{:});
    opts = parser.Results;

    visibilityText = "off";
    if logical(opts.Visible)
        visibilityText = "on";
    end

    figureHandle = figure( ...
        "Color", "w", ...
        "Visible", visibilityText, ...
        "Name", "GEN3 LITE Paper Tables", ...
        "Position", [120, 120, 1120, 720]);
    axesHandle = axes("Parent", figureHandle, ...
        "Position", [0.02, 0.02, 0.96, 0.96]);
    hold(axesHandle, "on");
    axis(axesHandle, [0.0, 1.0, 0.0, 1.0]);
    axis(axesHandle, "off");

    draw_dh_table(axesHandle, summary.dh_table);
    draw_joint_limit_table(axesHandle, summary.joint_table);

    imagePath = "";
    if strlength(string(opts.SavePath)) > 0
        imagePath = string(opts.SavePath);
        exportgraphics(axesHandle, imagePath, "Resolution", 240);
    end

    out = struct();
    out.figure_handle = figureHandle;
    out.axes_handle = axesHandle;
    out.image_path = imagePath;
end

function draw_dh_table(axesHandle, dhTable)
%DRAW_DH_TABLE Draw the upper classical-DH parameter table.
    leftX = 0.05;
    rightX = 0.95;
    titleY = 0.955;
    topRuleY = 0.93;
    headerSplitY = 0.86;
    bottomRuleY = 0.43;

    text(axesHandle, 0.50, titleY, "表 2.1 经典 D-H 参数表", ...
        "HorizontalAlignment", "center", ...
        "FontName", "SimSun", ...
        "FontSize", 18);

    draw_horizontal_rule(axesHandle, leftX, rightX, topRuleY, 2.2);
    draw_horizontal_rule(axesHandle, leftX, rightX, headerSplitY, 0.8);
    draw_horizontal_rule(axesHandle, leftX, rightX, bottomRuleY, 2.2);

    columnX = [0.11, 0.28, 0.45, 0.65, 0.86];
    headerY = 0.895;
    rowY = linspace(0.81, 0.49, height(dhTable));

    place_table_text(axesHandle, columnX(1), headerY, "i", 15);
    place_table_text(axesHandle, columnX(2), headerY, "\alpha_i", 15);
    place_table_text(axesHandle, columnX(3), headerY, "a_i / mm", 15);
    place_table_text(axesHandle, columnX(4), headerY, "d_i / mm", 15);
    place_table_text(axesHandle, columnX(5), headerY, "\theta_i", 15);

    for rowIndex = 1:height(dhTable)
        place_table_text(axesHandle, columnX(1), rowY(rowIndex), sprintf("%d", dhTable.index(rowIndex)), 16);
        place_table_text(axesHandle, columnX(2), rowY(rowIndex), dhTable.alpha_text(rowIndex), 16);
        place_table_text(axesHandle, columnX(3), rowY(rowIndex), dhTable.a_text_mm(rowIndex), 16);
        place_table_text(axesHandle, columnX(4), rowY(rowIndex), insert_spacing_around_plus(dhTable.d_text_mm(rowIndex)), 16);
        place_table_text(axesHandle, columnX(5), rowY(rowIndex), dhTable.theta_text(rowIndex), 16);
    end
end

function draw_joint_limit_table(axesHandle, jointTable)
%DRAW_JOINT_LIMIT_TABLE Draw the lower official joint-angle-range table.
    leftX = 0.06;
    rightX = 0.94;
    titleY = 0.335;
    topRuleY = 0.31;
    midRuleY = 0.25;
    bottomRuleY = 0.18;

    text(axesHandle, 0.50, titleY, "表 2.2 关节角范围", ...
        "HorizontalAlignment", "center", ...
        "FontName", "SimSun", ...
        "FontSize", 18);

    draw_horizontal_rule(axesHandle, leftX, rightX, topRuleY, 2.2);
    draw_horizontal_rule(axesHandle, leftX, rightX, midRuleY, 0.8);
    draw_horizontal_rule(axesHandle, leftX, rightX, bottomRuleY, 2.2);

    columnX = [0.11, 0.24, 0.38, 0.52, 0.66, 0.80, 0.93];
    headerY = 0.275;
    valueY = 0.215;

    place_table_text(axesHandle, columnX(1), headerY, "序号", 14);
    for jointIndex = 1:6
        place_table_text(axesHandle, columnX(jointIndex + 1), headerY, sprintf("%d", jointIndex), 14);
    end

    place_table_text(axesHandle, columnX(1), valueY, "\theta_i 范围 / deg", 10);
    for jointIndex = 1:6
        valueText = "[" + format_deg(jointTable.lower_deg(jointIndex)) + "," + ...
            format_deg(jointTable.upper_deg(jointIndex)) + "]";
        place_table_text(axesHandle, columnX(jointIndex + 1), valueY, valueText, 10);
    end
end

function draw_horizontal_rule(axesHandle, leftX, rightX, yValue, lineWidth)
%DRAW_HORIZONTAL_RULE Draw one horizontal table rule.
    plot(axesHandle, [leftX, rightX], [yValue, yValue], ...
        "k-", "LineWidth", lineWidth, "HandleVisibility", "off");
end

function place_table_text(axesHandle, xValue, yValue, textValue, fontSize)
%PLACE_TABLE_TEXT Draw centered table text with consistent styling.
    if nargin < 5
        fontSize = 15;
    end
    text(axesHandle, xValue, yValue, string(textValue), ...
        "HorizontalAlignment", "center", ...
        "VerticalAlignment", "middle", ...
        "Interpreter", "tex", ...
        "FontName", "SimSun", ...
        "FontSize", fontSize);
end

function textOut = insert_spacing_around_plus(textIn)
%INSERT_SPACING_AROUND_PLUS Improve readability of grouped length sums.
    textOut = regexprep(string(textIn), '\+', ' + ');
end

function textOut = format_deg(valueDeg)
%FORMAT_DEG Format one angle in degrees for paper-style display.
    textOut = compact_decimal(valueDeg);
end

function textOut = compact_decimal(value)
%COMPACT_DECIMAL Format with cleanup of trailing zeros.
    if abs(value) < 5e-4
        value = 0.0;
    end
    textOut = string(sprintf("%.2f", value));
    textOut = regexprep(textOut, '(\.\d*?)0+$', '$1');
    textOut = regexprep(textOut, '\.$', '');
    if textOut == "-0"
        textOut = "0";
    end
end
