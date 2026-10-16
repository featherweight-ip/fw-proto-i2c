#!/usr/bin/env bash
#****************************************************************************
#* ci/run.sh -- the checks CI runs on every commit, on either forge.
#*
#* Both workflow files (.github/workflows/ci.yml, .forgejo/workflows/ci.yml)
#* call this and nothing else to do the work, so what a check IS lives in one
#* place and the two forges cannot drift on it. It runs unchanged on a laptop,
#* which is how it gets debugged. fwvip-wb's ci/run.sh is the same shape.
#*
#*   1. bootstrap -- ivpm into a venv, then `ivpm update` of the dep-set
#*   2. dfm run <each check in CI_CHECKS> -- archetype verbs (project.dv), each
#*      with its own --report bundle; all always run, so one failing does not
#*      hide another
#*   3. collect   -- the CTRF, JUnit and SARIF files into $CI_REPORTS, and one
#*      verdict line per check
#*
#* Exit status is nonzero iff any check failed (or bootstrap did).
#*
#* Environment:
#*   CI_CHECKS    the dfm roots to run, space-separated (default:
#*                "smoke lint-rtl fv")
#*   CI_DEP_SET   ivpm dep-set to fetch (default: dev)
#*   CI_REPORTS   where the logs are collected (default: ci-reports)
#*   IVPM_CACHE   ivpm's package cache (default: ~/.cache/ivpm, the path the
#*                workflows cache between runs)
#*   CI_CACHE_DAYS
#*                prune cache entries not linked for this many days
#*                (default: 14; see ci/ivpm-cache-key.sh)
#*   CI_SKIP_BOOTSTRAP=1
#*                use packages/ as it is -- on a laptop that already has it,
#*                this is just the checks
#****************************************************************************
set -u -o pipefail

root="$(cd "$(dirname "$0")/.." && pwd)"
cd "$root"

dep_set="${CI_DEP_SET:-dev}"
reports="${CI_REPORTS:-ci-reports}"
export IVPM_CACHE="${IVPM_CACHE:-$HOME/.cache/ivpm}"
mkdir -p "$IVPM_CACHE"

group()    { echo "::group::$*"; }
endgroup() { echo "::endgroup::"; }

# ---- 1. bootstrap ----------------------------------------------------------
if [ "${CI_SKIP_BOOTSTRAP:-0}" != "1" ]; then
    group "bootstrap: ivpm update -d $dep_set"
    if ! command -v direnv >/dev/null 2>&1; then
        # packages/packages.envrc is written for direnv (source_env, PATH_add),
        # so the environment a developer gets from `direnv allow` is the one CI
        # gets too.
        sudo=""
        [ "$(id -u)" = "0" ] || sudo="sudo"
        $sudo apt-get update -qq && $sudo apt-get install -y -qq direnv >/dev/null \
            || { echo "::error::could not install direnv"; exit 1; }
    fi
    # uv as well as ivpm: without uv, ivpm installs python content with pip,
    # whose editable install writes egg-info into the package -- and a cached
    # package (yosys) is a read-only directory, so it fails.
    python3 -m venv .ci-venv \
        && .ci-venv/bin/pip install -q --upgrade pip ivpm uv \
        || { echo "::error::could not install ivpm"; exit 1; }
    .ci-venv/bin/ivpm --version 2>/dev/null || true
    # -a: clone git URLs as written (HTTPS). A runner has no ssh identity.
    PATH="$root/.ci-venv/bin:$PATH" .ci-venv/bin/ivpm update -a -d "$dep_set" \
        || { echo "::error::ivpm update -d $dep_set failed"; exit 1; }
    # Drop cache entries no run has linked for a while, so the saved cache
    # does not grow without bound.
    .ci-venv/bin/ivpm cache clean --days "${CI_CACHE_DAYS:-14}" \
        || echo "::warning::ivpm cache clean failed; the cache is unchanged"
    direnv allow . || exit 1
    endgroup
fi

# ---- 2. the checks ----------------------------------------------------------
rm -rf "$reports"
mkdir -p "$reports/ctrf"

declare -A status
run_check() {
    local verb="$1"
    group "dfm run $verb"
    direnv exec . dfm run "$verb" -u log --report "$reports/$verb"
    status[$verb]=$?
    endgroup
    # The report, readable in the log on a forge that renders no job summary.
    if [ -f "$reports/$verb/report.md" ]; then
        group "report: $verb (status ${status[$verb]})"
        cat "$reports/$verb/report.md"
        endgroup
    fi
}

read -r -a checks <<< "${CI_CHECKS:-smoke lint-rtl fv}"
for c in "${checks[@]}"; do run_check "$c"; done

# ---- 3. collect -------------------------------------------------------------
# Task rundirs are named for the task, not the verb: lint-rtl's work is done by
# this project's `lint` task.
copy_first() {
    local pattern="$1" dest="$2" f
    f=$(ls -1 $pattern 2>/dev/null | head -1)
    [ -n "$f" ] && mkdir -p "$dest" && cp "$f" "$dest"
}
copy_first "rundir/*.lint/lint-ctrf.json" "$reports/ctrf/"
copy_first "rundir/*.lint/lint.sarif"     "$reports/lint-rtl/"
copy_first "rundir/*.smoke/ctrf.json"     "$reports/smoke/"
copy_first "rundir/*.smoke/junit.xml"     "$reports/smoke/"

verdict() { [ "${status[$1]}" = "0" ] && echo "pass" || echo "FAIL (status ${status[$1]})"; }

if [ -n "${GITHUB_STEP_SUMMARY:-}" ]; then
    for verb in "${checks[@]}"; do
        {
            echo "<details><summary><code>dfm run $verb</code>: $(verdict $verb)</summary>"
            echo
            cat "$reports/$verb/report.md" 2>/dev/null || echo "_no report written_"
            echo
            echo "</details>"
            echo
        } >> "$GITHUB_STEP_SUMMARY"
    done
fi

rc=0
for c in "${checks[@]}"; do
    printf '%-13s %s\n' "$c:" "$(verdict $c)"
    [ "${status[$c]}" = "0" ] || rc=1
done
exit $rc
