#!/usr/bin/env bash
# Copyright (c) 2026, IsaacLab Tactile Array contributors.
# SPDX-License-Identifier: BSD-3-Clause

set -Eeuo pipefail

readonly EXPECTED_ISAACLAB_TAG="v2.3.2"
readonly EXPECTED_ISAACLAB_COMMIT="37ddf626871758333d6ed89cf64ad702aef127d0"
readonly CONTRIB_DISTRIBUTION="isaaclab_contrib"
readonly ADDON_DISTRIBUTION="isaaclab-tactile-array"
readonly ADDON_EXTRA="demo"
readonly CHECK_SUCCESS_TOKEN="__ISAACLAB_TACTILE_ARRAY_CHECK_OK__"
readonly -a REQUIRED_LFS_ASSETS=(
    "source/isaaclab_tactile_array/isaaclab_tactile_array/assets/tactile_array_finger.usd"
    "docs/media/tactile_array_demo.gif"
)

INSTALLER_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd -P)"
ADDON_DIR="${INSTALLER_DIR}/source/isaaclab_tactile_array"
ISAACLAB_ROOT=""
CHECK_ONLY=false

print_help() {
    cat <<'EOF'
Install the Tactile Array add-on into an official Isaac Lab v2.3.2 environment.

Usage:
  ./install.sh --isaaclab /path/to/IsaacLab
  ./install.sh /path/to/IsaacLab
  ISAACLAB_PATH=/path/to/IsaacLab ./install.sh
  ./install.sh --check --isaaclab /path/to/IsaacLab

Options:
  --isaaclab PATH   Path to an official Isaac Lab checkout at tag v2.3.2.
  --check           Validate the checkout and installed packages without changing anything.
  -h, --help        Show this help and exit.

If no path is supplied, the installer searches the current directory and the
add-on repository's immediate sibling directories. If more than one matching
v2.3.2 checkout is found, pass --isaaclab explicitly.

Prerequisite:
  Isaac Sim must already be available to the selected Isaac Lab checkout (for
  example through an active environment or its _isaac_sim link). The installer
  never modifies or copies files into the official Isaac Lab checkout. If this
  add-on was cloned without LFS objects, Git LFS is needed to fetch its USD/GIF.
EOF
}

log() {
    printf '[tactile-array] %s\n' "$*"
}

fail() {
    printf '[tactile-array] ERROR: %s\n' "$*" >&2
    exit 1
}

canonical_directory() {
    local directory="$1"
    [[ -d "${directory}" ]] || return 1
    (cd -- "${directory}" && pwd -P)
}

checkout_commit() {
    local directory="$1"
    git -C "${directory}" rev-parse --verify HEAD^{commit} 2>/dev/null
}

is_expected_checkout() {
    local directory="$1"
    local commit=""

    [[ -f "${directory}/isaaclab.sh" ]] || return 1
    commit="$(checkout_commit "${directory}")" || return 1
    [[ "${commit}" == "${EXPECTED_ISAACLAB_COMMIT}" ]]
}

validate_checkout() {
    local directory="$1"
    local commit=""
    local description="unknown"

    command -v git >/dev/null 2>&1 || fail "git is required to validate Isaac Lab ${EXPECTED_ISAACLAB_TAG}."
    [[ -d "${directory}" ]] || fail "Isaac Lab directory does not exist: ${directory}"
    [[ -f "${directory}/isaaclab.sh" ]] || fail "Not an Isaac Lab checkout (isaaclab.sh is missing): ${directory}"
    [[ -f "${directory}/source/isaaclab/setup.py" ]] \
        || fail "The checkout is missing source/isaaclab/setup.py: ${directory}"
    [[ -f "${directory}/source/isaaclab_contrib/setup.py" ]] \
        || fail "The checkout is missing source/isaaclab_contrib/setup.py: ${directory}"

    commit="$(checkout_commit "${directory}")" \
        || fail "Unable to read the Git commit from: ${directory}"
    if [[ "${commit}" != "${EXPECTED_ISAACLAB_COMMIT}" ]]; then
        description="$(git -C "${directory}" describe --tags --always --dirty 2>/dev/null || printf 'unknown')"
        fail "Expected official Isaac Lab ${EXPECTED_ISAACLAB_TAG} (${EXPECTED_ISAACLAB_COMMIT}), found ${description} (${commit})."
    fi

    if ! git -C "${directory}" diff --quiet --ignore-submodules -- 2>/dev/null \
        || ! git -C "${directory}" diff --cached --quiet --ignore-submodules -- 2>/dev/null; then
        log "WARNING: the Isaac Lab checkout has tracked local changes; this installer will leave them untouched."
    fi
}

add_candidate() {
    local candidate="$1"
    local resolved=""
    local existing=""

    resolved="$(canonical_directory "${candidate}" 2>/dev/null)" || return 0
    is_expected_checkout "${resolved}" || return 0
    for existing in "${DISCOVERED_CHECKOUTS[@]-}"; do
        [[ "${existing}" == "${resolved}" ]] && return 0
    done
    DISCOVERED_CHECKOUTS+=("${resolved}")
}

discover_checkout() {
    local candidate=""
    local current_directory=""
    local -a DISCOVERED_CHECKOUTS=()

    current_directory="$(pwd -P)"
    add_candidate "${current_directory}"
    add_candidate "${INSTALLER_DIR}/../IsaacLab"
    add_candidate "${INSTALLER_DIR}/../IsaacLab-v2.3.2"

    for candidate in "${INSTALLER_DIR}"/../*; do
        [[ -d "${candidate}" ]] || continue
        add_candidate "${candidate}"
    done

    case "${#DISCOVERED_CHECKOUTS[@]}" in
        0)
            fail "Could not find an official Isaac Lab ${EXPECTED_ISAACLAB_TAG} checkout. Pass --isaaclab PATH."
            ;;
        1)
            ISAACLAB_ROOT="${DISCOVERED_CHECKOUTS[0]}"
            ;;
        *)
            printf '[tactile-array] Multiple Isaac Lab %s checkouts were found:\n' "${EXPECTED_ISAACLAB_TAG}" >&2
            for candidate in "${DISCOVERED_CHECKOUTS[@]}"; do
                printf '  %s\n' "${candidate}" >&2
            done
            fail "Select one with --isaaclab PATH."
            ;;
    esac
}

parse_arguments() {
    local supplied_path=""

    while (($# > 0)); do
        case "$1" in
            --isaaclab)
                (($# >= 2)) || fail "--isaaclab requires a path."
                [[ -z "${supplied_path}" ]] || fail "The Isaac Lab path was supplied more than once."
                supplied_path="$2"
                shift 2
                ;;
            --isaaclab=*)
                [[ -z "${supplied_path}" ]] || fail "The Isaac Lab path was supplied more than once."
                supplied_path="${1#*=}"
                [[ -n "${supplied_path}" ]] || fail "--isaaclab requires a non-empty path."
                shift
                ;;
            --check)
                CHECK_ONLY=true
                shift
                ;;
            -h|--help)
                print_help
                exit 0
                ;;
            --)
                shift
                (($# <= 1)) || fail "Only one positional Isaac Lab path is accepted."
                if (($# == 1)); then
                    [[ -z "${supplied_path}" ]] || fail "The Isaac Lab path was supplied more than once."
                    supplied_path="$1"
                    shift
                fi
                ;;
            -*)
                fail "Unknown option: $1 (use --help for usage)"
                ;;
            *)
                [[ -z "${supplied_path}" ]] || fail "Only one Isaac Lab path is accepted."
                supplied_path="$1"
                shift
                ;;
        esac
    done

    if [[ -n "${supplied_path}" ]]; then
        ISAACLAB_ROOT="$(canonical_directory "${supplied_path}")" \
            || fail "Isaac Lab directory does not exist: ${supplied_path}"
    elif [[ -n "${ISAACLAB_PATH:-}" ]]; then
        ISAACLAB_ROOT="$(canonical_directory "${ISAACLAB_PATH}")" \
            || fail "ISAACLAB_PATH is not a directory: ${ISAACLAB_PATH}"
    else
        discover_checkout
    fi
}

validate_addon_source() {
    [[ -d "${ADDON_DIR}" ]] || fail "Add-on source directory is missing: ${ADDON_DIR}"
    if [[ ! -f "${ADDON_DIR}/setup.py" && ! -f "${ADDON_DIR}/pyproject.toml" ]]; then
        fail "Add-on packaging metadata is missing from: ${ADDON_DIR}"
    fi
}

run_isaaclab_python() {
    local wrapper_term="${TERM:-xterm}"

    # v2.3.2 runs `tabs 4` under `set -e`; TERM=dumb makes its launcher abort
    # before it reaches Python in non-interactive shells and CI.
    [[ "${wrapper_term}" == "dumb" ]] && wrapper_term="xterm"
    TERM="${wrapper_term}" bash "${ISAACLAB_ROOT}/isaaclab.sh" -p "$@"
}

is_lfs_pointer() {
    local path="$1"
    local first_line=""

    [[ -f "${path}" ]] || return 1
    IFS= read -r first_line < "${path}" || true
    [[ "${first_line}" == "version https://git-lfs.github.com/spec/v1" ]]
}

find_unavailable_assets() {
    local relative_path=""
    local absolute_path=""

    UNAVAILABLE_ASSETS=()
    for relative_path in "${REQUIRED_LFS_ASSETS[@]}"; do
        absolute_path="${INSTALLER_DIR}/${relative_path}"
        if [[ ! -s "${absolute_path}" ]] || is_lfs_pointer "${absolute_path}"; then
            UNAVAILABLE_ASSETS+=("${relative_path}")
        fi
    done
}

print_unavailable_assets() {
    local relative_path=""

    printf '[tactile-array] Required Git LFS assets are missing, empty, or still pointers:\n' >&2
    for relative_path in "${UNAVAILABLE_ASSETS[@]}"; do
        printf '  %s\n' "${relative_path}" >&2
    done
}

ensure_required_assets() {
    local allow_pull="$1"
    local include_paths=""
    local -a UNAVAILABLE_ASSETS=()

    find_unavailable_assets
    ((${#UNAVAILABLE_ASSETS[@]} == 0)) && return 0

    if [[ "${allow_pull}" != true ]]; then
        print_unavailable_assets
        fail "Run the installer without --check to fetch the required LFS objects."
    fi

    print_unavailable_assets
    if ! git lfs version >/dev/null 2>&1; then
        fail "Git LFS is not available. Install git-lfs, then rerun this installer (or run 'git lfs pull' in ${INSTALLER_DIR})."
    fi

    include_paths="$(IFS=,; printf '%s' "${REQUIRED_LFS_ASSETS[*]}")"
    log "Fetching required USD/GIF objects with Git LFS..."
    if ! git -C "${INSTALLER_DIR}" lfs pull --include="${include_paths}" --exclude=""; then
        fail "Git LFS could not fetch the required assets. Check network/repository access and rerun the installer."
    fi

    find_unavailable_assets
    if ((${#UNAVAILABLE_ASSETS[@]} > 0)); then
        print_unavailable_assets
        fail "Git LFS completed, but one or more required assets were not materialized."
    fi
}

# Isaac Lab v2.3.2's wrapper does not reliably propagate the child Python exit
# status. The Python probe therefore emits a token only after every check passes.
check_editable_distribution() {
    local distribution="$1"
    local expected_source="$2"
    local required_extra="${3:-}"
    local probe_output=""
    local probe_code='import json
import sys
from importlib import metadata
from pathlib import Path
from urllib.parse import unquote, urlparse

from packaging.requirements import Requirement

name, expected_text, required_extra, success_token = sys.argv[1:5]
expected = Path(expected_text).resolve()
dist = metadata.distribution(name)

direct_url_text = dist.read_text("direct_url.json")
if not direct_url_text:
    raise RuntimeError(f"{name} is installed, but not from an editable local checkout")
direct_url = json.loads(direct_url_text)
parsed_url = urlparse(direct_url.get("url", ""))
installed_source = Path(unquote(parsed_url.path)).resolve()
if parsed_url.scheme != "file" or installed_source != expected:
    raise RuntimeError(
        f"{name} points to {installed_source}, expected editable source {expected}"
    )
if not direct_url.get("dir_info", {}).get("editable", False):
    raise RuntimeError(f"{name} is not installed in editable mode")

if required_extra:
    provided_extras = set(dist.metadata.get_all("Provides-Extra") or [])
    if required_extra not in provided_extras:
        raise RuntimeError(f"{name} does not declare the [{required_extra}] extra")

    missing = []
    incompatible = []
    for requirement_text in dist.requires or []:
        requirement = Requirement(requirement_text)
        if requirement.marker is not None and not requirement.marker.evaluate({"extra": required_extra}):
            continue
        try:
            installed_version = metadata.version(requirement.name)
        except metadata.PackageNotFoundError:
            missing.append(requirement.name)
            continue
        if requirement.specifier and installed_version not in requirement.specifier:
            incompatible.append(f"{requirement.name}=={installed_version} (needs {requirement.specifier})")
    if missing:
        raise RuntimeError("missing demo dependencies: " + ", ".join(sorted(set(missing))))
    if incompatible:
        raise RuntimeError("incompatible demo dependencies: " + ", ".join(incompatible))

print(success_token)'

    probe_output="$(
        run_isaaclab_python -c "${probe_code}" \
            "${distribution}" "${expected_source}" "${required_extra}" "${CHECK_SUCCESS_TOKEN}" 2>&1
    )" || true

    if [[ "${probe_output}" == *"${CHECK_SUCCESS_TOKEN}"* ]]; then
        return 0
    fi

    printf '%s\n' "${probe_output}" >&2
    return 1
}

run_checks() {
    log "Checking the official Isaac Lab installation..."
    check_editable_distribution "isaaclab" "${ISAACLAB_ROOT}/source/isaaclab" \
        || fail "Isaac Lab is not installed from the selected ${EXPECTED_ISAACLAB_TAG} checkout. Run this installer without --check."

    log "Checking the official Isaac Lab contrib package..."
    check_editable_distribution "${CONTRIB_DISTRIBUTION}" "${ISAACLAB_ROOT}/source/isaaclab_contrib" \
        || fail "Isaac Lab contrib is not installed from the selected ${EXPECTED_ISAACLAB_TAG} checkout. Run this installer without --check."

    log "Checking the Tactile Array add-on and [${ADDON_EXTRA}] dependencies..."
    check_editable_distribution "${ADDON_DISTRIBUTION}" "${ADDON_DIR}" "${ADDON_EXTRA}" \
        || fail "The Tactile Array add-on is not fully installed. Run this installer without --check."

    log "OK: Isaac Lab ${EXPECTED_ISAACLAB_TAG}, ${CONTRIB_DISTRIBUTION}, and ${ADDON_DISTRIBUTION}[${ADDON_EXTRA}] are installed from the expected sources."
}

install_packages() {
    if check_editable_distribution "isaaclab" "${ISAACLAB_ROOT}/source/isaaclab" >/dev/null 2>&1; then
        log "Isaac Lab core already points to the selected ${EXPECTED_ISAACLAB_TAG} checkout."
    else
        log "Installing the official Isaac Lab core from ${ISAACLAB_ROOT}..."
        run_isaaclab_python -m pip install --no-build-isolation --editable \
            "${ISAACLAB_ROOT}/source/isaaclab"
        check_editable_distribution "isaaclab" "${ISAACLAB_ROOT}/source/isaaclab" \
            || fail "The official Isaac Lab core installation did not complete successfully."
    fi

    if check_editable_distribution "${CONTRIB_DISTRIBUTION}" "${ISAACLAB_ROOT}/source/isaaclab_contrib" >/dev/null 2>&1; then
        log "Isaac Lab contrib already points to the selected ${EXPECTED_ISAACLAB_TAG} checkout."
    else
        log "Installing the official Isaac Lab contrib package from ${ISAACLAB_ROOT}..."
        run_isaaclab_python -m pip install --no-build-isolation --editable \
            "${ISAACLAB_ROOT}/source/isaaclab_contrib"
        check_editable_distribution "${CONTRIB_DISTRIBUTION}" "${ISAACLAB_ROOT}/source/isaaclab_contrib" \
            || fail "The official Isaac Lab contrib installation did not complete successfully."
    fi

    log "Installing ${ADDON_DISTRIBUTION}[${ADDON_EXTRA}] from ${ADDON_DIR}..."
    run_isaaclab_python -m pip install --no-build-isolation --editable \
        "${ADDON_DIR}[${ADDON_EXTRA}]"

    run_checks
    log "Installation complete. No files in the official Isaac Lab checkout were patched."
}

main() {
    parse_arguments "$@"
    validate_checkout "${ISAACLAB_ROOT}"
    validate_addon_source

    log "Using official Isaac Lab ${EXPECTED_ISAACLAB_TAG}: ${ISAACLAB_ROOT}"
    log "Using Tactile Array add-on source: ${ADDON_DIR}"

    if [[ "${CHECK_ONLY}" == true ]]; then
        ensure_required_assets false
        run_checks
    else
        ensure_required_assets true
        install_packages
    fi
}

main "$@"
