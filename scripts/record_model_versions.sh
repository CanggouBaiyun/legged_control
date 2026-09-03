#!/usr/bin/env bash
set -euo pipefail

project_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
unitree_ros_dir="$project_root/third_party/unitree_ros"
unitree_mujoco_dir="$project_root/third_party/unitree_mujoco"
versions_file="$project_root/third_party/model_versions.txt"

if [[ ! -d "$unitree_ros_dir/.git" ]]; then
  printf 'Missing %s; follow docs/setup_guide.md first.\n' "$unitree_ros_dir" >&2
  exit 1
fi
if [[ ! -d "$unitree_mujoco_dir/.git" ]]; then
  printf 'Missing %s; follow docs/setup_guide.md first.\n' "$unitree_mujoco_dir" >&2
  exit 1
fi

unitree_ros_commit="$(git -C "$unitree_ros_dir" rev-parse HEAD)"
unitree_mujoco_commit="$(git -C "$unitree_mujoco_dir" rev-parse HEAD)"

{
  printf 'unitree_ros %s\n' "$unitree_ros_commit"
  printf 'unitree_mujoco %s\n' "$unitree_mujoco_commit"
} > "$versions_file"

cat "$versions_file"

