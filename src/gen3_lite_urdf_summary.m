function summary = gen3_lite_urdf_summary(varargin)
%GEN3_LITE_URDF_SUMMARY Summarize GEN3 LITE zero-pose kinematics from URDF.
%
%   SUMMARY = GEN3_LITE_URDF_SUMMARY() parses the GEN3 LITE URDF shipped in
%   kortex_description and returns:
%     - zero-pose frame locations/orientations for base, joints, and tool
%     - revolute joint angle limits in rad/deg
%     - the official classical DH table published for GEN3 LITE
%
%   SUMMARY = GEN3_LITE_URDF_SUMMARY("UrdfPath", PATH) uses a custom URDF.
%
%   Assumptions:
%     - the URDF contains a single serial 6R GEN3 LITE chain
%     - revolute joints appear in kinematic order in the URDF file
%     - the fixed joint named END_EFFECTOR is the tool offset frame
%     - DH and joint-limit outputs follow the official manual values

    parser = inputParser;
    parser.addParameter("UrdfPath", "", @(x) ischar(x) || isstring(x));
    parser.parse(varargin{:});

    % Resolve the default GEN3 LITE URDF inside kortex_description.
    repoRoot = fileparts(fileparts(mfilename("fullpath")));
    urdfPath = string(parser.Results.UrdfPath);
    if strlength(urdfPath) == 0
        urdfPath = fullfile(repoRoot, "kortex_description", "arms", ...
            "gen3_lite", "6dof", "urdf", "GEN3-LITE.urdf");
    end
    if ~isfile(urdfPath)
        error("HCDR:URDFNotFound", "GEN3 LITE URDF not found: %s", urdfPath);
    end

    % Parse all joints directly from the XML document to avoid toolbox
    % dependencies in the summary pipeline.
    xmlDocument = xmlread(char(urdfPath));
    robotNode = xmlDocument.getDocumentElement();
    robotName = string(char(robotNode.getAttribute("name")));
    jointNodes = robotNode.getElementsByTagName("joint");

    revoluteJoints = repmat(empty_joint_record(), 0, 1);
    toolFixedJoint = empty_joint_record();
    toolJointFound = false;

    for nodeIndex = 0:(jointNodes.getLength() - 1)
        jointNode = jointNodes.item(nodeIndex);
        jointInfo = parse_joint_node(jointNode);
        jointType = jointInfo.type;

        if jointType == "revolute"
            revoluteJoints(end + 1, 1) = jointInfo; %#ok<AGROW>
        elseif jointType == "fixed" && jointInfo.name == "END_EFFECTOR"
            toolFixedJoint = jointInfo;
            toolJointFound = true;
        end
    end

    if numel(revoluteJoints) ~= 6
        error("HCDR:URDFUnexpectedJointCount", ...
            "Expected 6 revolute joints in GEN3 LITE URDF, got %d.", ...
            numel(revoluteJoints));
    end
    if ~toolJointFound
        error("HCDR:URDFMissingToolJoint", ...
            "Fixed joint END_EFFECTOR was not found in %s.", urdfPath);
    end

    % Build the exact zero-pose transform chain from the URDF joint origins.
    [frameTable, frameTransforms, jointAxesWorld, toolTransform] = ...
        build_zero_pose_frames(revoluteJoints, toolFixedJoint);

    % Assemble the raw joint-origin table and the official classical DH rows.
    jointTable = build_joint_table(revoluteJoints, jointAxesWorld);
    dhTable = build_dh_table();

    summary = struct();
    summary.robot_name = robotName;
    summary.urdf_path = string(urdfPath);
    summary.revolute_joint_count = double(numel(revoluteJoints));
    summary.joints = revoluteJoints;
    summary.joint_table = jointTable;
    summary.frame_table = frameTable;
    summary.frame_transforms = frameTransforms;
    summary.tool_fixed_name = toolFixedJoint.name;
    summary.tool_fixed_xyz_m = double(toolFixedJoint.origin_xyz(:));
    summary.tool_fixed_rpy_rad = double(toolFixedJoint.origin_rpy(:));
    summary.tool_transform = toolTransform;
    summary.dh_table = dhTable;
end

function jointInfo = empty_joint_record()
%EMPTY_JOINT_RECORD Return a typed empty joint struct.
    jointInfo = struct( ...
        "name", "", ...
        "type", "", ...
        "parent_link", "", ...
        "child_link", "", ...
        "origin_xyz", zeros(3, 1, "double"), ...
        "origin_rpy", zeros(3, 1, "double"), ...
        "origin_transform", eye(4, "double"), ...
        "axis_xyz", [0.0; 0.0; 1.0], ...
        "lower_rad", NaN, ...
        "upper_rad", NaN);
end

function jointInfo = parse_joint_node(jointNode)
%PARSE_JOINT_NODE Read one URDF joint entry from the XML DOM.
    jointInfo = empty_joint_record();
    jointInfo.name = string(char(jointNode.getAttribute("name")));
    jointInfo.type = string(char(jointNode.getAttribute("type")));

    % Read direct child tags so parent/child/origin/limit stay explicit.
    childNodes = jointNode.getChildNodes();
    for childIndex = 0:(childNodes.getLength() - 1)
        childNode = childNodes.item(childIndex);
        if childNode.getNodeType() ~= childNode.ELEMENT_NODE
            continue;
        end

        childName = string(char(childNode.getNodeName()));
        switch childName
            case "origin"
                jointInfo.origin_xyz = read_vector_attribute(childNode, "xyz", [0.0; 0.0; 0.0]);
                jointInfo.origin_rpy = read_vector_attribute(childNode, "rpy", [0.0; 0.0; 0.0]);
            case "parent"
                jointInfo.parent_link = string(char(childNode.getAttribute("link")));
            case "child"
                jointInfo.child_link = string(char(childNode.getAttribute("link")));
            case "axis"
                jointInfo.axis_xyz = read_vector_attribute(childNode, "xyz", [0.0; 0.0; 1.0]);
            case "limit"
                jointInfo.lower_rad = read_scalar_attribute(childNode, "lower", NaN);
                jointInfo.upper_rad = read_scalar_attribute(childNode, "upper", NaN);
        end
    end

    jointInfo.origin_transform = make_transform(jointInfo.origin_xyz, jointInfo.origin_rpy);
end

function vectorValue = read_vector_attribute(node, attributeName, defaultValue)
%READ_VECTOR_ATTRIBUTE Parse a three-element URDF attribute into double.
    attributeText = strtrim(string(char(node.getAttribute(attributeName))));
    if strlength(attributeText) == 0
        vectorValue = double(defaultValue(:));
        return;
    end

    parsedValues = sscanf(char(attributeText), "%f");
    if numel(parsedValues) ~= 3
        error("HCDR:URDFMalformedVector", ...
            "Expected 3 values in attribute %s, got %d.", attributeName, numel(parsedValues));
    end
    vectorValue = double(parsedValues(:));
end

function scalarValue = read_scalar_attribute(node, attributeName, defaultValue)
%READ_SCALAR_ATTRIBUTE Parse one scalar URDF attribute into double.
    attributeText = strtrim(string(char(node.getAttribute(attributeName))));
    if strlength(attributeText) == 0
        scalarValue = double(defaultValue);
        return;
    end
    scalarValue = double(str2double(attributeText));
end

function transform = make_transform(xyzM, rpyRad)
%MAKE_TRANSFORM Build a 4x4 transform from URDF xyz/rpy values.
    rotation = rotz3(rpyRad(3)) * roty3(rpyRad(2)) * rotx3(rpyRad(1));
    transform = eye(4, "double");
    transform(1:3, 1:3) = rotation;
    transform(1:3, 4) = double(xyzM(:));
end

function [frameTable, frameTransforms, jointAxesWorld, toolTransform] = ...
        build_zero_pose_frames(revoluteJoints, toolFixedJoint)
%BUILD_ZERO_POSE_FRAMES Propagate URDF zero-pose transforms along the chain.
    jointCount = numel(revoluteJoints);
    frameCount = jointCount + 2;  % BASE + six joint frames + TOOL.

    frameNames = strings(frameCount, 1);
    parentNames = strings(frameCount, 1);
    originX = zeros(frameCount, 1, "double");
    originY = zeros(frameCount, 1, "double");
    originZ = zeros(frameCount, 1, "double");
    zAxisX = zeros(frameCount, 1, "double");
    zAxisY = zeros(frameCount, 1, "double");
    zAxisZ = zeros(frameCount, 1, "double");
    frameTransforms = zeros(4, 4, frameCount, "double");
    jointAxesWorld = zeros(3, jointCount, "double");

    % Base frame is the URDF BASE link frame.
    currentTransform = eye(4, "double");
    frameNames(1) = "BASE";
    parentNames(1) = "";
    frameTransforms(:, :, 1) = currentTransform;
    zAxisX(1) = 0.0;
    zAxisY(1) = 0.0;
    zAxisZ(1) = 1.0;

    % Each child link frame at q=0 is the corresponding joint frame.
    for jointIndex = 1:jointCount
        currentTransform = currentTransform * revoluteJoints(jointIndex).origin_transform;
        frameSlot = jointIndex + 1;

        frameNames(frameSlot) = revoluteJoints(jointIndex).name;
        parentNames(frameSlot) = revoluteJoints(jointIndex).parent_link;
        frameTransforms(:, :, frameSlot) = currentTransform;

        originVector = currentTransform(1:3, 4);
        jointAxisWorld = currentTransform(1:3, 1:3) * revoluteJoints(jointIndex).axis_xyz;
        jointAxisWorld = jointAxisWorld / max(norm(jointAxisWorld), eps);

        originX(frameSlot) = originVector(1);
        originY(frameSlot) = originVector(2);
        originZ(frameSlot) = originVector(3);
        zAxisX(frameSlot) = jointAxisWorld(1);
        zAxisY(frameSlot) = jointAxisWorld(2);
        zAxisZ(frameSlot) = jointAxisWorld(3);
        jointAxesWorld(:, jointIndex) = jointAxisWorld;
    end

    % The tool frame is the fixed offset after the sixth revolute joint.
    toolTransform = currentTransform * toolFixedJoint.origin_transform;
    frameNames(end) = "TOOL";
    parentNames(end) = toolFixedJoint.parent_link;
    frameTransforms(:, :, end) = toolTransform;
    originX(end) = toolTransform(1, 4);
    originY(end) = toolTransform(2, 4);
    originZ(end) = toolTransform(3, 4);
    zAxisX(end) = toolTransform(1, 3);
    zAxisY(end) = toolTransform(2, 3);
    zAxisZ(end) = toolTransform(3, 3);

    frameTable = table( ...
        frameNames, parentNames, originX, originY, originZ, zAxisX, zAxisY, zAxisZ, ...
        'VariableNames', { ...
            'name', 'parent_name', 'x_m', 'y_m', 'z_m', ...
            'z_axis_x', 'z_axis_y', 'z_axis_z'});
end

function jointTable = build_joint_table(revoluteJoints, jointAxesWorld)
%BUILD_JOINT_TABLE Collect raw URDF revolute joint metadata into one table.
    jointCount = numel(revoluteJoints);
    jointNames = strings(jointCount, 1);
    parentLinks = strings(jointCount, 1);
    childLinks = strings(jointCount, 1);
    originX = zeros(jointCount, 1, "double");
    originY = zeros(jointCount, 1, "double");
    originZ = zeros(jointCount, 1, "double");
    rollRad = zeros(jointCount, 1, "double");
    pitchRad = zeros(jointCount, 1, "double");
    yawRad = zeros(jointCount, 1, "double");
    axisLocalX = zeros(jointCount, 1, "double");
    axisLocalY = zeros(jointCount, 1, "double");
    axisLocalZ = zeros(jointCount, 1, "double");
    axisWorldX = zeros(jointCount, 1, "double");
    axisWorldY = zeros(jointCount, 1, "double");
    axisWorldZ = zeros(jointCount, 1, "double");
    lowerRad = zeros(jointCount, 1, "double");
    upperRad = zeros(jointCount, 1, "double");

    for jointIndex = 1:jointCount
        jointInfo = revoluteJoints(jointIndex);
        jointNames(jointIndex) = jointInfo.name;
        parentLinks(jointIndex) = jointInfo.parent_link;
        childLinks(jointIndex) = jointInfo.child_link;
        originX(jointIndex) = jointInfo.origin_xyz(1);
        originY(jointIndex) = jointInfo.origin_xyz(2);
        originZ(jointIndex) = jointInfo.origin_xyz(3);
        rollRad(jointIndex) = jointInfo.origin_rpy(1);
        pitchRad(jointIndex) = jointInfo.origin_rpy(2);
        yawRad(jointIndex) = jointInfo.origin_rpy(3);
        axisLocalX(jointIndex) = jointInfo.axis_xyz(1);
        axisLocalY(jointIndex) = jointInfo.axis_xyz(2);
        axisLocalZ(jointIndex) = jointInfo.axis_xyz(3);
        axisWorldX(jointIndex) = jointAxesWorld(1, jointIndex);
        axisWorldY(jointIndex) = jointAxesWorld(2, jointIndex);
        axisWorldZ(jointIndex) = jointAxesWorld(3, jointIndex);
        lowerRad(jointIndex) = jointInfo.lower_rad;
        upperRad(jointIndex) = jointInfo.upper_rad;
    end

    % Use the official actuator-limit sheet supplied by the user instead of
    % the URDF limits so exported tables/figures match the official manual.
    [officialLowerDeg, officialUpperDeg] = official_gen3_lite_joint_limits_deg();
    lowerRad = deg2rad(officialLowerDeg);
    upperRad = deg2rad(officialUpperDeg);

    jointTable = table( ...
        jointNames, parentLinks, childLinks, ...
        originX, originY, originZ, rollRad, pitchRad, yawRad, ...
        axisLocalX, axisLocalY, axisLocalZ, ...
        axisWorldX, axisWorldY, axisWorldZ, ...
        lowerRad, upperRad, officialLowerDeg, officialUpperDeg, ...
        'VariableNames', { ...
            'name', 'parent_link', 'child_link', ...
            'origin_x_m', 'origin_y_m', 'origin_z_m', ...
            'roll_rad', 'pitch_rad', 'yaw_rad', ...
            'axis_local_x', 'axis_local_y', 'axis_local_z', ...
            'axis_world_x', 'axis_world_y', 'axis_world_z', ...
            'lower_rad', 'upper_rad', 'lower_deg', 'upper_deg'});
end

function [lowerDeg, upperDeg] = official_gen3_lite_joint_limits_deg()
%OFFICIAL_GEN3_LITE_JOINT_LIMITS_DEG Return official actuator limits in deg.
    lowerDeg = [-154.1; -150.1; -150.1; -148.98; -144.97; -148.98];
    upperDeg = [ 154.1;  150.1;  150.1;  148.98;  145.00;  148.98];
end

function dhTable = build_dh_table()
%BUILD_DH_TABLE Return the official GEN3 LITE classical DH parameter table.
    jointNames = ["J0"; "J1"; "J2"; "J3"; "J4"; "J5"];
    alphaRad = [pi/2; pi; pi/2; pi/2; pi/2; 0.0];
    aM = [0.0; 0.2800; 0.0; 0.0; 0.0; 0.0];
    dM = [0.2433; 0.0300; 0.0200; 0.2450; 0.0570; 0.2350];
    thetaOffsetRad = [0.0; pi/2; pi/2; pi/2; pi; pi/2];

    alphaText = ["\pi/2"; "\pi"; "\pi/2"; "\pi/2"; "\pi/2"; "0"];
    aTextMm = ["0.0"; "280.0"; "0.0"; "0.0"; "0.0"; "0.0"];
    dTextMm = ["(128.3+115.0)"; "30.0"; "20.0"; "(140.0+105.0)"; "(28.5+28.5)"; "(105.0+130.0)"];
    thetaText = ["q_1"; "q_2 + \pi/2"; "q_3 + \pi/2"; "q_4 + \pi/2"; "q_5 + \pi"; "q_6 + \pi/2"];

    dhTable = table( ...
        (1:6).', jointNames, ...
        alphaRad, rad2deg(alphaRad), alphaText(:), ...
        aM, aTextMm(:), ...
        dM, dTextMm(:), ...
        thetaOffsetRad, rad2deg(thetaOffsetRad), thetaText(:), ...
        'VariableNames', { ...
            'index', 'name', ...
            'alpha_rad', 'alpha_deg', 'alpha_text', ...
            'a_m', 'a_text_mm', ...
            'd_m', 'd_text_mm', ...
            'theta_offset_rad', 'theta_offset_deg', 'theta_text'});
end

function rotation = rotx3(angleRad)
%ROTX3 3x3 rotation matrix about the x-axis.
    rotation = [ ...
        1.0, 0.0, 0.0; ...
        0.0, cos(angleRad), -sin(angleRad); ...
        0.0, sin(angleRad),  cos(angleRad)];
end

function rotation = roty3(angleRad)
%ROTY3 3x3 rotation matrix about the y-axis.
    rotation = [ ...
         cos(angleRad), 0.0, sin(angleRad); ...
         0.0,           1.0, 0.0; ...
        -sin(angleRad), 0.0, cos(angleRad)];
end

function rotation = rotz3(angleRad)
%ROTZ3 3x3 rotation matrix about the z-axis.
    rotation = [ ...
        cos(angleRad), -sin(angleRad), 0.0; ...
        sin(angleRad),  cos(angleRad), 0.0; ...
        0.0,            0.0,           1.0];
end
