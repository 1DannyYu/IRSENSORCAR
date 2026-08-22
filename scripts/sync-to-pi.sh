#!/usr/bin/env bash
set -euo pipefail

# Commit local repository changes, push main, then fast-forward the Raspberry Pi.
# Rebuildable/generated working folders are deliberately excluded from the commit.

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
REMOTE="${REMOTE:-danny}"
PI_HOST="${PI_HOST:-carpi}"
PI_REPO="${PI_REPO:-\$HOME/Car-and-Robotic-Arm}"
BRANCH="${BRANCH:-main}"
COMMIT_MESSAGE="$(date '+%Y-%m-%d %H:%M:%S %z')"

cd "$REPO_ROOT"

if [[ "$(git branch --show-current)" != "$BRANCH" ]]; then
    echo "ERROR: expected local branch '$BRANCH'." >&2
    exit 1
fi

if ! git diff --quiet --exit-code --cached; then
    echo "ERROR: the index already contains staged changes. Commit or unstage them first." >&2
    git diff --cached --name-status >&2
    exit 1
fi

git add -A -- \
    . \
    ':!output' \
    ':!output/**' \
    ':!tmp' \
    ':!tmp/**' \
    ':!scratch' \
    ':!scratch/**'

if git diff --cached --quiet --exit-code; then
    echo "No changes to commit."
    exit 0
fi

echo "Changes to commit:"
git diff --cached --name-status
git commit -m "$COMMIT_MESSAGE"
git push "$REMOTE" "$BRANCH"

ssh "$PI_HOST" "cd \"$PI_REPO\" && \
    if test -n \"\$(git status --porcelain --untracked-files=no)\"; then \
        echo 'ERROR: Raspberry Pi has uncommitted tracked changes; pull cancelled.'; \
        git status --short --untracked-files=no; \
        exit 1; \
    fi && \
    git pull --ff-only origin '$BRANCH' && \
    echo \"PI HEAD=\$(git rev-parse --short HEAD)\""

echo "Local HEAD=$(git rev-parse --short HEAD)"
