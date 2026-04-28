function texTables = gen3_lite_urdf_tex_tables(summary)
%GEN3_LITE_URDF_TEX_TABLES Build copy-ready TeX tables for GEN3 LITE.
%
%   TEXTABLES = GEN3_LITE_URDF_TEX_TABLES(SUMMARY) converts the official
%   DH table and joint-limit table in SUMMARY into LaTeX table code.

    arguments
        summary (1, 1) struct
    end

    dhTable = summary.dh_table;
    jointTable = summary.joint_table;
    jointCount = height(dhTable);

    dhLines = strings(jointCount + 9, 1);
    dhLines(1) = "\begin{table}[htbp]";
    dhLines(2) = "\centering";
    dhLines(3) = "\caption{GEN3 LITE Classical DH Parameters}";
    dhLines(4) = "\begin{tabular}{ccccc}";
    dhLines(5) = "\hline";
    dhLines(6) = "$i$ & $\alpha_i$ & $a_i$ (mm) & $d_i$ (mm) & $\theta_i$ \\";
    dhLines(7) = "\hline";
    for jointIndex = 1:jointCount
        dhLines(7 + jointIndex) = sprintf( ...
            '%d & $%s$ & %s & $%s$ & $%s$ \\\\', ...
            dhTable.index(jointIndex), ...
            char(dhTable.alpha_text(jointIndex)), ...
            char(dhTable.a_text_mm(jointIndex)), ...
            char(insert_spacing_around_plus(dhTable.d_text_mm(jointIndex))), ...
            char(dhTable.theta_text(jointIndex)));
    end
    dhLines(end - 1) = "\hline";
    dhLines(end) = "\end{tabular}" + newline + "\end{table}";

    rangeHeader = "$i$";
    rangeValues = "$\theta_i$ range ($^\circ$)";
    for jointIndex = 1:jointCount
        rangeHeader = rangeHeader + sprintf(" & %d", jointIndex);
        rangeValues = rangeValues + sprintf(" & [%s,%s]", ...
            format_decimal_deg(jointTable.lower_deg(jointIndex)), ...
            format_decimal_deg(jointTable.upper_deg(jointIndex)));
    end
    rangeHeader = rangeHeader + " \\";
    rangeValues = rangeValues + " \\";

    limitLines = strings(10, 1);
    limitLines(1) = "\begin{table}[htbp]";
    limitLines(2) = "\centering";
    limitLines(3) = "\caption{GEN3 LITE Joint Limits}";
    limitLines(4) = sprintf("\\begin{tabular}{c%s}", repmat('c', 1, jointCount));
    limitLines(5) = "\hline";
    limitLines(6) = rangeHeader;
    limitLines(7) = "\hline";
    limitLines(8) = rangeValues;
    limitLines(9) = "\hline";
    limitLines(10) = "\end{tabular}" + newline + "\end{table}";

    texTables = struct();
    texTables.dh_table_tex = strjoin(dhLines, newline);
    texTables.joint_limit_tex = strjoin(limitLines, newline);
    texTables.combined_text = texTables.dh_table_tex + newline + newline + texTables.joint_limit_tex;
end

function textOut = insert_spacing_around_plus(textIn)
%INSERT_SPACING_AROUND_PLUS Improve readability of grouped length sums.
    textOut = regexprep(string(textIn), '\+', ' + ');
end

function textOut = format_decimal_deg(angleDeg)
%FORMAT_DECIMAL_DEG Format one angle in degrees for TeX copy.
    textOut = format_decimal_generic(angleDeg);
end

function textOut = format_decimal_generic(value)
%FORMAT_DECIMAL_GENERIC Format a scalar with compact trailing-zero cleanup.
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
