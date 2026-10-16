#!/usr/bin/env bash
#****************************************************************************
#* ci/ivpm-cache-key.sh -- print a cache key for what $IVPM_CACHE now holds.
#*
#* Both workflows save the ivpm cache under this key after ci/run.sh. It
#* names the cache's CONTENTS, not the manifest: ivpm stores every entry as
#* <cache>/<package>/<version>/, so the list of those directories says exactly
#* what is in it. A key from hashFiles('ivpm.yaml') does not change when
#* edapack publishes a new `latest` tool, so the restored cache would miss on
#* every run and never be re-saved.
#*
#* The ISO week is part of the key so the cache is re-saved at least weekly.
#* That carries the entries' refreshed last-linked times, which is what
#* `ivpm cache clean` in ci/run.sh ages entries by; without it an entry in
#* daily use would look unused and be pruned from the saved copy.
#*
#* Prints nothing (and the workflows skip the save) if the cache is empty.
#****************************************************************************
set -u -o pipefail

cache="${IVPM_CACHE:-$HOME/.cache/ivpm}"
[ -d "$cache" ] || exit 0

# Entries only: not the .meta.json sidecars (mutable), nor an in-flight or
# crashed publish (*.staging.*) or an evicted entry awaiting deletion (.gc.*).
entries=$(find "$cache" -mindepth 2 -maxdepth 2 -type d \
            ! -name '*.staging.*' ! -name '.gc.*' -printf '%P\n' | LC_ALL=C sort)
[ -n "$entries" ] || exit 0

echo "$(date -u +%G-W%V)-$(printf '%s\n' "$entries" | sha256sum | cut -c1-16)"
