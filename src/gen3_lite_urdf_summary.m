function summary = gen3_lite_urdf_summary(varargin)
%GEN3_LITE_URDF_SUMMARY Summarize GEN3 LITE zero-pose kinematics from URDF.
%
%   SUMMARY = GEN3_LITE_URDF_SUMMARY() parses the GEN3 LITE URDF shipped in
%   kortex_description and returns:
%     - zero-pose frame locations/orientations for base, joints, and tool
%     - revolute joint angle limits in rad/deg
%     - a DH-style summary table fitted from each URDF joint origin
%
%   SUMMARY = GEN3_LITE_URDF_SUMMARY("UrdfPath", PATH) uses a custom URDF.
%
%   Assumptions:
%     - the URDF contains a single serial 6R GEN3 LITE chain
%     - revolute joints appear in kinematic order in the URDF file
%     - the fixed joint named END_EFFECTOR is the tool offset frame
%
%   Edge cases:
%     - if a joint transform is not exactly representable by one standard DH
%       row, the function still returns the best-fit row plus residual terms

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

    % Assemble the raw joint-origin table and a DH-style best-fit digest.
    jointTable = build_joint_table(revoluteJoints, jointAxesWorld);
    dhTable = build_dh_table(revoluteJoints);

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

    jointTable = table( ...
        jointNames, parentLinks, childLinks, ...
        originX, originY, originZ, rollRad, pitchRad, yawRad, ...
        axisLocalX, axisLocalY, axisLocalZ, ...
        axisWorldX, axisWorldY, axisWorldZ, ...
        lowerRad, upperRad, rad2deg(lowerRad), rad2deg(upperRad), ...
        'VariableNames', { ...
            'name', 'parent_link', 'child_link', ...
            'origin_x_m', 'origin_y_m', 'origin_z_m', ...
            'roll_rad', 'pitch_rad', 'yaw_rad', ...
            'axis_local_x', 'axis_local_y', 'axis_local_z', ...
            'axis_world_x', 'axis_world_y', 'axis_world_z', ...
            'lower_rad', 'upper_rad', 'lower_deg', 'upper_deg'});
end

function dhTable = build_dh_table(revoluteJoints)
%BUILD_DH_TABLE Fit one standard-DH row to each URDF joint origin transform.
    jointCount = numel(revoluteJoints);
    jointNames = strings(jointCount, 1);
    aM = zeros(jointCount, 1, "double");
    alphaRad = zeros(jointCount, 1, "double");
    dM = zeros(jointCount, 1, "double");
    thetaOffsetRad = zeros(jointCount, 1, "double");
    fitError = zeros(jointCount, 1, "double");
    positionResidualM = zeros(jointCount, 1, "double");
    rotationResidualDeg = zeros(jointCount, 1, "double");

    for jointIndex = 1:jointCount
        jointNames(jointIndex) = revoluteJoints(jointIndex).name;
        [params, metrics] = fit_standard_dh_row(revoluteJoints(jointIndex).origin_transform);
        aM(jointIndex) = params(1);
        alphaRad(jointIndex) = params(2);
        dM(jointIndex) = params(3);
        thetaOffsetRad(jointIndex) = params(4);
        fitError(jointIndex) = metrics.total_error;
        positionResidualM(jointIndex) = metrics.position_residual_m;
        rotationResidualDeg(jointIndex) = rad2deg(metrics.rotation_residual_rad);
    end

    dhTable = table( ...
        jointNames, aM, alphaRad, rad2deg(alphaRad), dM, ...
        thetaOffsetRad, rad2deg(thetaOffsetRad), ...
        fitError, positionResidualM, rotationResidualDeg, ...
        'VariableNames', { ...
            'name', 'a_m', 'alpha_rad', 'alpha_deg', 'd_m', ...
            'theta_offset_rad', 'theta_offset_deg', ...
            'fit_error', 'position_residual_m', 'rotation_residual_deg'});
end

function [bestParams, metrics] = fit_standard_dh_row(targetTransform)
%FIT_STANDARD_DH_ROW Fit [a alpha d theta0] to one URDF zero-pose transform.
    translation = targetTransform(1:3, 4);
    xyNorm = hypot(translation(1), translation(2));

    % The initial guess comes from the raw translation and rotation terms.
    initialTheta = atan2(translation(2), translation(1));
    if xyNorm < 1e-12
        initialTheta = atan2(targetTransform(2, 1), targetTransform(1, 1));
    end
    initialGuess = [ ...
        xyNorm, ...
        atan2(targetTransform(3, 2), targetTransform(3, 3)), ...
        translation(3), ...
        initialTheta];

    options = optimset("Display", "off", "TolX", 1e-10, "TolFun", 1e-10, ...
        "MaxFunEvals", 4000, "MaxIter", 4000);
    objective = @(params) dh_fit_cost(params, targetTransform);
    bestParams = fminsearch(objective, initialGuess, options);

    bestParams(2) = wrap_to_pi(bestParams(2));
    bestParams(4) = wrap_to_pi(bestParams(4));

    fittedTransform = standard_dh_transform(bestParams(1), bestParams(2), bestParams(3), bestParams(4));
    positionResidual = targetTransform(1:3, 4) - fittedTransform(1:3, 4);
    relativeRotation = fittedTransform(1:3, 1:3).' * targetTransform(1:3, 1:3);

    metrics = struct();
    metrics.total_error = objective(bestParams);
    metrics.position_residual_m = norm(positionResidual);
    metrics.rotation_residual_rad = rotation_matrix_angle(relativeRotation);
end

function cost = dh_fit_cost(params, targetTransform)
%DH_FIT_COST Weighted transform mismatch for one standard-DH row.
    fittedTransform = standard_dh_transform(params(1), params(2), params(3), params(4));
    deltaPosition = targetTransform(1:3, 4) - fittedTransform(1:3, 4);
    deltaRotation = targetTransform(1:3, 1:3) - fittedTransform(1:3, 1:3);

    % Translation is in meters while rotation is unitless; increase the
    % position weight so centimeter-scale offsets remain visible.
    cost = 50.0 * dot(deltaPosition, deltaPosition) + sum(deltaRotation(:) .^ 2);
end

function transform = standard_dh_transform(aM, alphaRad, dM, thetaOffsetRad)
%STANDARD_DH_TRANSFORM Standard DH homogeneous transform at zero joint angle.
    cosTheta = cos(thetaOffsetRad);
    sinTheta = sin(thetaOffsetRad);
    cosAlpha = cos(alphaRad);
    sinAlpha = sin(alphaRad);

    transform = [ ...
        cosTheta, -sinTheta * cosAlpha,  sinTheta * sinAlpha, aM * cosTheta; ...
        sinTheta,  cosTheta * cosAlpha, -cosTheta * sinAlpha, aM * sinTheta; ...
        0.0,       sinAlpha,             cosAlpha,            dM; ...
        0.0,       0.0,                  0.0,                 1.0];
end

function angleRad = rotation_matrix_angle(rotationMatrix)
%ROTATION_MATRIX_ANGLE Return the principal angle of a 3x3 rotation matrix.
    traceValue = max(min((trace(rotationMatrix) - 1.0) / 2.0, 1.0), -1.0);
    angleRad = acos(traceValue);
end

function wrappedAngle = wrap_to_pi(angleRad)
%WRAP_TO_PI Wrap angles into [-pi, pi].
    wrappedAngle = mod(angleRad + pi, 2.0 * pi) - pi;
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
