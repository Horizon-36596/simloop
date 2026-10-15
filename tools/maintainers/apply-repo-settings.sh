#!/usr/bin/env bash
#
# Applies the repository settings in docs/MAINTAINERS.md to Horizon-36596/simloop with the GitHub CLI.
# Run it ONCE, by a repository admin, AFTER the repository is public: a private repository on the free
# plan cannot have branch rules, so most of this would fail before then.
#
# Usage (Git Bash on Windows, or any bash):   bash tools/maintainers/apply-repo-settings.sh --yes
#
# It is safe to run again: every step sets a value rather than adding one, and the branch ruleset is
# updated in place if it already exists.

set -euo pipefail

repo=${SIMLOOP_REPO:-Horizon-36596/simloop}

if [ "${1:-}" != "--yes" ]; then
    echo "This changes settings on $repo. Read docs/MAINTAINERS.md, then run it again with --yes."
    exit 1
fi

visibility=$(gh api "repos/$repo" --jq .visibility)
if [ "$visibility" != "public" ]; then
    echo "$repo is $visibility. Make it public first; branch rules need a public repository on this plan."
    exit 1
fi

echo "1/6  Branch ruleset on main: PR + 1 code-owner review, required checks, no force push, no bypass"
ruleset=$(cat <<'JSON'
{
  "name": "main",
  "target": "branch",
  "enforcement": "active",
  "bypass_actors": [],
  "conditions": { "ref_name": { "include": ["~DEFAULT_BRANCH"], "exclude": [] } },
  "rules": [
    { "type": "deletion" },
    { "type": "non_fast_forward" },
    { "type": "pull_request",
      "parameters": {
        "required_approving_review_count": 1,
        "require_code_owner_review": true,
        "dismiss_stale_reviews_on_push": true,
        "require_last_push_approval": false,
        "required_review_thread_resolution": false
      } },
    { "type": "required_status_checks",
      "parameters": {
        "strict_required_status_checks_policy": false,
        "required_status_checks": [
          { "context": "test" },
          { "context": "leak-gate" },
          { "context": "secret-scan" }
        ]
      } }
  ]
}
JSON
)
existing=$(gh api "repos/$repo/rulesets" --jq '.[] | select(.name == "main") | .id' | head -n 1)
if [ -n "$existing" ]; then
    printf '%s' "$ruleset" | gh api -X PUT "repos/$repo/rulesets/$existing" --input - > /dev/null
else
    printf '%s' "$ruleset" | gh api -X POST "repos/$repo/rulesets" --input - > /dev/null
fi

echo "2/6  Discussions on"
gh api -X PATCH "repos/$repo" -F has_discussions=true > /dev/null

echo "3/6  Private vulnerability reporting on"
gh api -X PUT "repos/$repo/private-vulnerability-reporting" > /dev/null

echo "4/6  Secret scanning and push protection on"
printf '%s' '{"security_and_analysis":{"secret_scanning":{"status":"enabled"},"secret_scanning_push_protection":{"status":"enabled"}}}' \
    | gh api -X PATCH "repos/$repo" --input - > /dev/null

echo "5/6  Actions: read-only default token; Pages built by Actions"
gh api -X PUT "repos/$repo/actions/permissions/workflow" \
    -f default_workflow_permissions=read -F can_approve_pull_request_reviews=false > /dev/null
if gh api "repos/$repo/pages" > /dev/null 2>&1; then
    gh api -X PUT "repos/$repo/pages" -f build_type=workflow > /dev/null
else
    gh api -X POST "repos/$repo/pages" -f build_type=workflow > /dev/null
fi

echo "6/6  The 'fidelity' label the sim-mismatch issue form uses"
gh label create fidelity --repo "$repo" --color 5319E7 \
    --description "The sim runs but does not match a real robot" --force > /dev/null

echo "Done. Check them at https://github.com/$repo/settings"
