#!/usr/bin/env bash
#
# Applies the repository settings in docs/MAINTAINERS.md to Horizon-36596/simloop with the GitHub CLI.
# Run it ONCE, by a repository admin, AFTER the repository is public: a private repository on the free
# plan cannot have branch rules, so most of this would fail before then.
#
# Usage (Git Bash on Windows, or any bash):   bash tools/maintainers/apply-repo-settings.sh --yes
#
# It is safe to run again: every step sets a value rather than adding one, and each ruleset is updated
# in place if one with its name already exists.

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

# Two rulesets on main. GitHub combines them, so a pull request must satisfy both:
#
#   checks  - everyone, the owner included, with no bypass at all: changes reach main only through a
#             pull request, and only once `test`, `leak-gate` and `secret-scan` pass on an up-to-date
#             branch. No force push, no deletion. The leak gate can never be skipped.
#   review  - 1 approving review from a code owner, stale approvals dismissed on a new push. Repository
#             admins may bypass it, but only through a pull request (bypass_mode "pull_request"), so a
#             direct push to main is still blocked by `checks`.
#
# Effect: an outside contributor's pull request needs a Horizon admin's approval; Horizon's own pull
# requests still need every check but not a second person. A ruleset cannot name a single user as a
# bypass actor (the API accepts Integration, OrganizationAdmin, RepositoryRole, Team and DeployKey), so
# the bypass is the repository Admin role, actor_id 5 - every repository admin, featurescript included.
#
# The check names are the job ids in .github/workflows/test.yml (test) and leak-gate.yml (leak-gate,
# secret-scan). Renaming a job means changing them here and re-running this script.
ruleset_checks=$(cat <<'JSON'
{
  "name": "checks",
  "target": "branch",
  "enforcement": "active",
  "bypass_actors": [],
  "conditions": { "ref_name": { "include": ["~DEFAULT_BRANCH"], "exclude": [] } },
  "rules": [
    { "type": "deletion" },
    { "type": "non_fast_forward" },
    { "type": "pull_request",
      "parameters": {
        "required_approving_review_count": 0,
        "require_code_owner_review": false,
        "dismiss_stale_reviews_on_push": false,
        "require_last_push_approval": false,
        "required_review_thread_resolution": false
      } },
    { "type": "required_status_checks",
      "parameters": {
        "strict_required_status_checks_policy": true,
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
ruleset_review=$(cat <<'JSON'
{
  "name": "review",
  "target": "branch",
  "enforcement": "active",
  "bypass_actors": [
    { "actor_id": 5, "actor_type": "RepositoryRole", "bypass_mode": "pull_request" }
  ],
  "conditions": { "ref_name": { "include": ["~DEFAULT_BRANCH"], "exclude": [] } },
  "rules": [
    { "type": "pull_request",
      "parameters": {
        "required_approving_review_count": 1,
        "require_code_owner_review": true,
        "dismiss_stale_reviews_on_push": true,
        "require_last_push_approval": false,
        "required_review_thread_resolution": false
      } }
  ]
}
JSON
)

# Creates the ruleset, or updates the one with the same name in place.
apply_ruleset() {
    local name=$1 body=$2 id
    id=$(gh api "repos/$repo/rulesets" --jq ".[] | select(.name == \"$name\") | .id" | head -n 1)
    if [ -n "$id" ]; then
        printf '%s' "$body" | gh api -X PUT "repos/$repo/rulesets/$id" --input - > /dev/null
    else
        printf '%s' "$body" | gh api -X POST "repos/$repo/rulesets" --input - > /dev/null
    fi
}

echo "1/6  Rulesets on main: 'checks' (PR + required checks, no bypass) and 'review' (code-owner approval, admins bypass via PR)"
apply_ruleset checks "$ruleset_checks"
apply_ruleset review "$ruleset_review"

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
