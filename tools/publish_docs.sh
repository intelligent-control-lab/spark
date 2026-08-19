#!/usr/bin/env bash
set -euo pipefail

script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
repo_root="$(cd -- "${script_dir}/.." && pwd)"
site_repo="${SPARK_DOCS_SITE_DIR:-${repo_root}/../spark-docs-site}"
build_dir="${repo_root}/docs/_build/html"

if [[ ! -d "${site_repo}/.git" ]]; then
  echo "Publishing checkout not found at ${site_repo}" >&2
  echo "Set SPARK_DOCS_SITE_DIR to the spark-docs-site checkout." >&2
  exit 1
fi

site_remote="$(git -C "${site_repo}" remote get-url origin)"
if [[ "${site_remote}" != *"spark-docs-site"* ]]; then
  echo "Refusing to synchronize into unexpected repository: ${site_remote}" >&2
  exit 1
fi

python -m sphinx -W --keep-going \
  -d "${repo_root}/docs/_build/doctrees" \
  -b html "${repo_root}/docs" "${build_dir}"

rsync -a --delete \
  --exclude='.git/' \
  --exclude='.doctrees/' \
  --exclude='CNAME' \
  --exclude='README.md' \
  "${build_dir}/" "${site_repo}/"

touch "${site_repo}/.nojekyll"
echo "Generated site synchronized to ${site_repo}"
echo "Review it with: git -C ${site_repo} status --short"
