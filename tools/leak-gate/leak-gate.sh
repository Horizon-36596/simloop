#!/usr/bin/env bash
#
# The public leak gate. Fails if this repository contains something a season-agnostic, public library
# must not: a dependency on a team's robot code, a real FTC game's name in the library source, someone's
# home directory, CAD or robot logs, keys, or any file over 1 MB. The rules are in patterns.txt and the
# reviewed exceptions in allowlist.txt, both beside this script.
#
# Usage:   bash tools/leak-gate/leak-gate.sh [repository-root]
#
# With no argument it checks the repository the current directory is in. It checks every tracked file
# plus every untracked file git does not ignore, so a file you have not added yet is caught too. Outside
# a git repository it checks every file under the root.
#
# Exit status: 0 clean, 1 at least one violation, 2 the gate itself could not run.
#
# Runs on Linux (CI) and on Windows in Git Bash. Uses bash, grep, awk, find, xargs and wc, nothing else.

set -u

here=$(cd "$(dirname "$0")" && pwd)
patterns=${LEAK_GATE_PATTERNS:-$here/patterns.txt}
allowlist=${LEAK_GATE_ALLOWLIST:-$here/allowlist.txt}

if [ $# -gt 0 ]; then
    root=$1
else
    root=$(git rev-parse --show-toplevel 2>/dev/null || pwd)
fi
cd "$root" || { echo "leak-gate: cannot enter $root" >&2; exit 2; }
[ -r "$patterns" ] || { echo "leak-gate: cannot read $patterns" >&2; exit 2; }
[ -r "$allowlist" ] || { echo "leak-gate: cannot read $allowlist" >&2; exit 2; }

work=$(mktemp -d 2>/dev/null || mktemp -d -t leakgate)
trap 'rm -rf "$work"' EXIT
files=$work/files
hits=$work/hits
used=$work/used
: > "$hits"
: > "$used"

# --- The files to check ------------------------------------------------------------------------------
if git rev-parse --is-inside-work-tree >/dev/null 2>&1; then
    git -c core.quotepath=off ls-files --cached --others --exclude-standard > "$work/listed" \
        || { echo "leak-gate: git ls-files failed" >&2; exit 2; }
else
    find . -type f ! -path './.git/*' | sed 's|^\./||' > "$work/listed"
fi
# Keep regular files only (a tracked file deleted in the working tree is not there to check).
while IFS= read -r f; do
    [ -f "$f" ] && printf '%s\n' "$f"
done < "$work/listed" | LC_ALL=C sort -u > "$files"
count=$(wc -l < "$files" | tr -d ' ')

# The gate's own rule and exception files spell out what they look for, so content rules skip them.
self='^tools/leak-gate/'

# Files in scope for one rule, NUL-separated for xargs.
in_scope() {
    grep -E -e "$1" "$files" | grep -v -E -e "$self" | tr '\n' '\0'
}

# Reads "path:line:text" (content rules) or "path" (path and size rules) on stdin and appends every
# line not covered by an allowlist entry to $hits as "rule-id  path:line: text".
filter() {
    LC_ALL=C awk -v id="$1" -v kind="$2" -v allowfile="$allowlist" -v usedfile="$used" '
        function trim(s) { sub(/\r$/, "", s); gsub(/^[ \t]+|[ \t]+$/, "", s); return s }
        BEGIN {
            while ((getline line < allowfile) > 0) {
                sub(/\r$/, "", line)
                if (line ~ /^[ \t]*(#|$)/) continue
                a = index(line, "|"); if (a == 0) continue
                rest = substr(line, a + 1); b = index(rest, "|"); if (b == 0) continue
                key = trim(substr(line, 1, a - 1)) SUBSEP substr(rest, 1, b - 1) SUBSEP trim(substr(rest, b + 1))
                allow[key] = line
            }
        }
        {
            if (kind == "content") {
                a = index($0, ":"); path = substr($0, 1, a - 1); rest = substr($0, a + 1)
                b = index(rest, ":"); lineno = substr(rest, 1, b - 1); text = trim(substr(rest, b + 1))
                where = path ":" lineno ": " text
            } else {
                path = $0; text = ""; where = path
            }
            key = id SUBSEP path SUBSEP text
            if (key in allow) { print allow[key] >> usedfile; next }
            print id "  " where
        }' >> "$hits"
}

# --- Run every rule ----------------------------------------------------------------------------------
rules=0
while read -r kind id scope flags pattern; do
    case "$kind" in ''|'#'*) continue ;; esac
    [ "$flags" = "-" ] && flags=""
    : > "$work/err"
    rules=$((rules + 1))
    case "$kind" in
        content)
            # -I skips binary files; grep exits 1 on no match, which xargs reports as 123. Neither is
            # an error here.
            in_scope "$scope" > "$work/scope"
            # With no file in scope, grep would read stdin instead, so skip it.
            [ -s "$work/scope" ] && xargs -0 grep -n -H -I -E $flags -e "$pattern" -- \
                < "$work/scope" 2>"$work/err" | filter "$id" content
            ;;
        path)
            grep -v -E -e "$self" "$files" | grep -E $flags -e "$pattern" | filter "$id" path
            ;;
        size)
            # find -size +Nc is "more than N bytes" (POSIX).
            in_scope "$scope" > "$work/scope"
            [ -s "$work/scope" ] && xargs -0 sh -c 'find "$@" -type f -size +'"$pattern"'c' sh \
                < "$work/scope" 2>"$work/err" | sed 's|^\./||' | filter "$id" size
            ;;
        *)
            echo "leak-gate: unknown rule kind '$kind' in $patterns" >&2
            exit 2
            ;;
    esac
    if [ -s "$work/err" ]; then
        echo "leak-gate: rule $id could not run:" >&2
        cat "$work/err" >&2
        exit 2
    fi
done < <(tr -d '\r' < "$patterns")

if [ "$rules" -eq 0 ]; then
    echo "leak-gate: no rules found in $patterns" >&2
    exit 2
fi

# --- Report ------------------------------------------------------------------------------------------
tr -d '\r' < "$allowlist" | grep -v -E '^[[:space:]]*(#|$)' > "$work/entries" || true
stale=$(LC_ALL=C sort -u "$used" | grep -v -x -F -f - "$work/entries" 2>/dev/null || true)
if [ -n "$stale" ]; then
    echo "leak-gate: these allowlist entries matched nothing and can be removed:"
    printf '%s\n' "$stale" | sed 's/^/  /'
    echo
fi

if [ -s "$hits" ]; then
    n=$(wc -l < "$hits" | tr -d ' ')
    echo "leak-gate: FAILED - $n violation(s) in $count files:"
    echo
    sed 's/^/  /' "$hits"
    echo
    echo "Rules are in tools/leak-gate/patterns.txt; what each one is for is written above it."
    echo "Remove the file or line. If it is genuinely safe, add an exact entry with a reason to"
    echo "tools/leak-gate/allowlist.txt and say why in your pull request."
    exit 1
fi

echo "leak-gate: passed - $rules rules, $count files, no violations."
exit 0
