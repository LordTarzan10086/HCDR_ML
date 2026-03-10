function setupInfo = hcdr_python_setup(opts)
%HCDR_PYTHON_SETUP Configure MATLAB-Python bridge for this workspace only.
%
%   setupInfo = HCDR_PYTHON_SETUP() ensures:
%   1) MATLAB Python interpreter is bound to a supported workspace env
%      when Python is not loaded yet.
%   2) Project root and src directories are available in py.sys.path so
%      local modules such as pin_terms.py are importable.
%
%   This function does not modify MATLAB global preferences and only affects
%   the current MATLAB session.
%
%   Name-Value inputs:
%   - python_executable: Explicit Python executable path. If empty, this
%     function tries environment variable HCDR_PYTHON_EXE, then common
%     conda env paths under USERPROFILE.
%   - verbose: Print setup summary/warnings.
%
%   Output:
%   - setupInfo: struct with fields
%     python_status, python_executable, desired_python_executable,
%     repo_root, src_dir, numpy_available, pinocchio_available.

    arguments
        opts.python_executable (1, 1) string = ""
        opts.verbose (1, 1) logical = false
    end

    repoRoot = fileparts(fileparts(mfilename("fullpath")));
    srcDir = fullfile(repoRoot, "src");
    desiredPythonExe = resolve_python_executable(opts.python_executable);

    % Stabilize embedded Python/Numpy/Pinocchio runtime on Windows MATLAB:
    % force single-thread MKL/OMP to avoid intermittent crash in batched calls.
    if strlength(string(getenv("OMP_NUM_THREADS"))) == 0
        setenv("OMP_NUM_THREADS", "1");
    end
    if strlength(string(getenv("MKL_NUM_THREADS"))) == 0
        setenv("MKL_NUM_THREADS", "1");
    end
    if strlength(string(getenv("NUMEXPR_NUM_THREADS"))) == 0
        setenv("NUMEXPR_NUM_THREADS", "1");
    end

    % Bind Python interpreter only when Python is not loaded yet.
    pyEnvironment = pyenv;
    if pyEnvironment.Status == "NotLoaded"
        if strlength(desiredPythonExe) == 0
            error("HCDR:PythonExeNotFound", ...
                "No workspace Python executable found. Set HCDR_PYTHON_EXE or install hcdr_pin.");
        end
        pyenv("Version", char(desiredPythonExe));
        pyEnvironment = pyenv;
    elseif strlength(desiredPythonExe) > 0 && ...
            ~strcmpi(string(pyEnvironment.Executable), desiredPythonExe) && opts.verbose
        warning("HCDR:PythonEnvMismatch", ...
            "Current pyenv executable differs from preferred workspace executable: %s", ...
            char(desiredPythonExe));
    end

    % Ensure local Python module lookup includes project root and src.
    py.importlib.import_module("sys");
    ensure_python_path(repoRoot);
    ensure_python_path(srcDir);

    % Report module availability for diagnostics.
    numpyAvailable = module_exists("numpy");
    pinocchioAvailable = module_exists("pinocchio");

    setupInfo = struct( ...
        "python_status", string(pyEnvironment.Status), ...
        "python_executable", string(pyEnvironment.Executable), ...
        "desired_python_executable", desiredPythonExe, ...
        "repo_root", string(repoRoot), ...
        "src_dir", string(srcDir), ...
        "numpy_available", logical(numpyAvailable), ...
        "pinocchio_available", logical(pinocchioAvailable));

    if opts.verbose
        fprintf("[hcdr_python_setup] pyenv=%s\n", char(setupInfo.python_executable));
        fprintf("[hcdr_python_setup] numpy=%d, pinocchio=%d\n", ...
            setupInfo.numpy_available, setupInfo.pinocchio_available);
    end
end

function desiredPythonExe = resolve_python_executable(explicitPythonExe)
%RESOLVE_PYTHON_EXECUTABLE Resolve preferred Python executable for HCDR.

    if strlength(explicitPythonExe) > 0
        if isfile(explicitPythonExe)
            desiredPythonExe = explicitPythonExe;
            return;
        end
        error("HCDR:PythonExeMissing", "Configured python_executable does not exist: %s", ...
            char(explicitPythonExe));
    end

    userProfile = string(getenv("USERPROFILE"));
    candidateList = [ ...
        string(getenv("HCDR_PYTHON_EXE")); ...
        string(fullfile(userProfile, "miniconda3", "envs", "hcdr_pin", "python.exe")); ...
        string(fullfile(userProfile, "miniconda3", "envs", "cdpr_env", "python.exe"))];

    desiredPythonExe = "";
    for k = 1:numel(candidateList)
        candidateExe = strtrim(candidateList(k));
        if strlength(candidateExe) > 0 && isfile(candidateExe)
            desiredPythonExe = candidateExe;
            return;
        end
    end
end

function ensure_python_path(pathText)
%ENSURE_PYTHON_PATH Prepend local path into py.sys.path if missing.

    pathText = char(pathText);
    if count(py.sys.path, pathText) == 0
        insert(py.sys.path, int32(0), pathText);
    end
end

function tf = module_exists(moduleName)
%MODULE_EXISTS True when a Python module can be discovered by importlib.

    moduleSpec = py.importlib.util.find_spec(char(moduleName));
    tf = ~isequal(moduleSpec, py.None);
end
