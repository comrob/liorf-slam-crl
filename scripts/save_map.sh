#!/usr/bin/env bash

set -euo pipefail

SERVICE_NAME="liorf/save_map"
SERVICE_TYPE="liorf/srv/SaveMap"
MAP_NODE_NAME="/liorf_mapOptimization"

# Script-level defaults (can be overridden via environment variables).
DEFAULT_RESOLUTION="${LIORF_SAVE_MAP_DEFAULT_RESOLUTION:-0}"
DEFAULT_DESTINATION="${LIORF_SAVE_MAP_DEFAULT_DESTINATION:-}"

get_save_pcd_directory() {
  local param_out
  if param_out=$(ros2 param get "$MAP_NODE_NAME" savePCDDirectory 2>/dev/null); then
    # Expected format: "String value is: /Downloads/LOAM/"
    echo "$param_out" | sed -n 's/^.*String value is: //p'
    return 0
  fi

  # Fallback to the built-in default in ParamServer.
  echo "/Downloads/LOAM/"
}

normalize_path() {
  local p="$1"
  if command -v realpath >/dev/null 2>&1; then
    realpath -m "$p"
  else
    echo "$p"
  fi
}

resolve_save_directory() {
  local input_destination="$1"
  local home_dir
  home_dir="${HOME:-/tmp}"

  if [[ -z "$input_destination" ]]; then
    local default_dir
    default_dir="$(get_save_pcd_directory)"
    default_dir="${default_dir%\"}"
    default_dir="${default_dir#\"}"
    if [[ "$default_dir" = /* ]]; then
      # mapOptimization treats savePCDDirectory as HOME-relative even if it starts with '/'.
      default_dir="${default_dir#/}"
    fi
    normalize_path "$home_dir/$default_dir"
    return 0
  fi

  if [[ "$input_destination" = /* ]]; then
    normalize_path "$input_destination"
    return 0
  fi

  if [[ "$input_destination" = ~/* ]]; then
    normalize_path "$home_dir/${input_destination#~/}"
    return 0
  fi

  normalize_path "$home_dir/$input_destination"
}

usage() {
  cat <<'EOF'
Usage: scripts/save_map.sh [-r RESOLUTION] [-d DESTINATION]

Calls the liorf map saving service.

Options:
  -r, --resolution   Map voxel resolution (float). Use 0 to keep original cloud density.
  -d, --destination  Save path passed to SaveMap.destination.
                     Path semantics follow mapOptimization:
                       /abs/path   -> absolute path
                       ~/path      -> HOME-expanded path
                       rel/path    -> relative to HOME
  -a, --absolute-path Absolute path shortcut (equivalent to --destination /abs/path).
  -h, --help         Show this help.

Defaults:
  resolution: ${LIORF_SAVE_MAP_DEFAULT_RESOLUTION:-0}
  destination: ${LIORF_SAVE_MAP_DEFAULT_DESTINATION:-<empty, use node savePCDDirectory>}

Environment overrides:
  LIORF_SAVE_MAP_DEFAULT_RESOLUTION
  LIORF_SAVE_MAP_DEFAULT_DESTINATION

Examples:
  scripts/save_map.sh
  scripts/save_map.sh -r 0.2
  scripts/save_map.sh -r 0.2 -d /tmp/liorf_map
  scripts/save_map.sh -a /tmp/liorf_map
  scripts/save_map.sh --destination ~/Downloads/LOAM
EOF
}

resolution="$DEFAULT_RESOLUTION"
destination="$DEFAULT_DESTINATION"
absolute_path=""

while [[ $# -gt 0 ]]; do
  case "$1" in
    -r|--resolution)
      if [[ $# -lt 2 ]]; then
        echo "Error: missing value for $1" >&2
        usage
        exit 1
      fi
      resolution="$2"
      shift 2
      ;;
    -d|--destination)
      if [[ $# -lt 2 ]]; then
        echo "Error: missing value for $1" >&2
        usage
        exit 1
      fi
      destination="$2"
      shift 2
      ;;
    -a|--absolute-path)
      if [[ $# -lt 2 ]]; then
        echo "Error: missing value for $1" >&2
        usage
        exit 1
      fi
      absolute_path="$2"
      shift 2
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    *)
      echo "Error: unknown argument '$1'" >&2
      usage
      exit 1
      ;;
  esac
done

if [[ -n "$absolute_path" && -n "$destination" ]]; then
  echo "Error: use either --destination or --absolute-path, not both." >&2
  exit 1
fi

if [[ -n "$absolute_path" ]]; then
  if [[ "$absolute_path" != /* ]]; then
    echo "Error: --absolute-path must start with '/'" >&2
    exit 1
  fi
  destination="$absolute_path"
fi

if ! command -v ros2 >/dev/null 2>&1; then
  echo "Error: 'ros2' command not found. Source your ROS environment first." >&2
  exit 1
fi

if [[ ! "$resolution" =~ ^[0-9]+([.][0-9]+)?$ ]]; then
  echo "Error: resolution must be a non-negative number, got '$resolution'" >&2
  exit 1
fi

# Escape for YAML double-quoted string.
escaped_destination=${destination//\\/\\\\}
escaped_destination=${escaped_destination//\"/\\\"}

echo "Calling ${SERVICE_NAME} (resolution=${resolution}, destination='${destination}')"
response="$(ros2 service call "${SERVICE_NAME}" "${SERVICE_TYPE}" \
  "{resolution: ${resolution}, destination: \"${escaped_destination}\"}")"
echo "$response"

if grep -q "success=True" <<<"$response"; then
  save_directory="$(echo "$response" | sed -n "s/.*save_directory='\([^']*\)'.*/\1/p")"
  enu_map_saved="$(echo "$response" | sed -n "s/.*enu_map_saved=\([^,)]*\).*/\1/p")"
  keyframes_used="$(echo "$response" | sed -n "s/.*keyframes_used=\([^,)]*\).*/\1/p")"
  surf_points_local="$(echo "$response" | sed -n "s/.*surf_points_local=\([^,)]*\).*/\1/p")"
  surf_points_enu="$(echo "$response" | sed -n "s/.*surf_points_enu=\([^,)]*\).*/\1/p")"
  global_points_local="$(echo "$response" | sed -n "s/.*global_points_local=\([^,)]*\).*/\1/p")"
  global_points_enu="$(echo "$response" | sed -n "s/.*global_points_enu=\([^,)]*\).*/\1/p")"
  message="$(echo "$response" | sed -n "s/.*message='\([^']*\)'.*/\1/p")"

  if [[ -z "$save_directory" ]]; then
    save_directory="$(resolve_save_directory "$destination")"
  fi

  if [[ -z "$destination" ]]; then
    default_dir_raw="$(get_save_pcd_directory)"
    default_dir_raw="${default_dir_raw%\"}"
    default_dir_raw="${default_dir_raw#\"}"
    echo "Destination source: node default savePCDDirectory='$default_dir_raw'"
    echo "Destination semantics: savePCDDirectory is treated as HOME-relative by map node for compatibility."
  else
    echo "Destination source: user argument"
  fi

  echo
  echo "Map saved to: $save_directory"
  [[ -n "$enu_map_saved" ]] && echo "ENU map saved: $enu_map_saved"
  [[ -n "$keyframes_used" ]] && echo "Keyframes used: $keyframes_used"
  [[ -n "$surf_points_local" ]] && echo "Local surf points: $surf_points_local"
  [[ -n "$surf_points_enu" ]] && echo "ENU surf points: $surf_points_enu"
  [[ -n "$global_points_local" ]] && echo "Local global-map points: $global_points_local"
  [[ -n "$global_points_enu" ]] && echo "ENU global-map points: $global_points_enu"
  [[ -n "$message" ]] && echo "Message: $message"
else
  echo
  echo "Map save did not report success; destination path not confirmed." >&2
  exit 1
fi
