% HCDR workspace-local startup bootstrap.
%
% This file is stored in the repository root so it only applies when MATLAB
% starts from this workspace. It does not write global MATLAB preferences.

projectRoot = fileparts(mfilename("fullpath"));
srcDir = fullfile(projectRoot, "src");

% Ensure project source directory is available for function resolution.
if isfolder(srcDir)
    addpath(srcDir);
end

% Best-effort Python bridge bootstrap for Pinocchio workflows.
if exist("hcdr_python_setup", "file") == 2
    try
        hcdr_python_setup("verbose", false);
    catch setupError
        warning("HCDR:StartupPython", ...
            "Workspace Python bootstrap skipped: %s", setupError.message);
    end
end
