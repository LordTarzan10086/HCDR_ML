function [enabled, reason] = hcdr_workspace_scan_enabled(cfg, opts)
%HCDR_WORKSPACE_SCAN_ENABLED Decide whether workspace scan should run.
%
%   [ENABLED, REASON] = HCDR_WORKSPACE_SCAN_ENABLED(CFG) centralizes the
%   v3.3 policy that workspace scanning is disabled by default and only
%   runs when explicitly enabled.
%
%   Decision order:
%   1) HCDR_SKIP_WORKSPACE=1  -> disabled
%   2) HCDR_ENABLE_WORKSPACE=1 -> enabled
%   3) cfg.enable_workspace_scan -> enabled/disabled
%   4) otherwise -> disabled
%
%   Name-Value:
%   - verbose: print standardized log line.
%   - context: free-form label used in the log output.

    arguments
        cfg (1, 1) struct = struct()
        opts.verbose (1, 1) logical = false
        opts.context (1, 1) string = "workspace"
    end

    skipEnv = strtrim(string(getenv("HCDR_SKIP_WORKSPACE")));
    enableEnv = strtrim(string(getenv("HCDR_ENABLE_WORKSPACE")));

    if strcmpi(skipEnv, "1")
        enabled = false;
        reason = "env_skip";
    elseif strcmpi(enableEnv, "1")
        enabled = true;
        reason = "env_enable";
    elseif isfield(cfg, "enable_workspace_scan") && ~isempty(cfg.enable_workspace_scan)
        enabled = logical(cfg.enable_workspace_scan);
        if enabled
            reason = "cfg_enable";
        else
            reason = "cfg_disable";
        end
    else
        enabled = false;
        reason = "default_disabled";
    end

    if opts.verbose
        if enabled
            fprintf("%s scan enabled (%s)\n", opts.context, reason);
        else
            fprintf("%s scan skipped (%s)\n", opts.context, reason);
        end
    end
end
