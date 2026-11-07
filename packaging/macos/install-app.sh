#!/usr/bin/env bash
# ==============================================================================
# Ogun — macOS Application Installer
# Installs Ogun.app into /Applications and configures LaunchServices & CLI link.
# ==============================================================================

# Running this with `source` would leak `set -u`/`set -e` into the caller's
# shell and kill it on the first error, so refuse and ask for direct execution.
if [[ "${BASH_SOURCE[0]:-$0}" != "$0" ]]; then
    echo "Error: run this script directly (./packaging/macos/install-app.sh), don't 'source' it." >&2
    return 1 2>/dev/null || exit 1
fi

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_DIR="$(cd "${SCRIPT_DIR}/../.." && pwd)"

# Defaults
DEFAULT_DEST_DIR="/Applications"
DEST_DIR="${DEFAULT_DEST_DIR}"
CLI_DIR="${HOME}/.local/bin"
INSTALL_CLI=true
FORCE_BUILD=false
UNINSTALL=false
SOURCE_BUNDLE=""

print_usage() {
    cat <<EOF
Usage: $(basename "$0") [OPTIONS] [PATH_TO_OGUN_APP]

Installs the Ogun STL Repair Tool application bundle into /Applications.

Arguments:
  PATH_TO_OGUN_APP         Optional path to Ogun.app bundle. If not specified,
                           the script looks in build_release/src/app/Ogun.app
                           (or build/src/app/Ogun.app), building it if needed.

Options:
  -b, --build              Force building the release target before installing.
  -d, --dest <dir>         Destination directory (default: /Applications).
      --cli-dir <dir>      Directory for the 'ogun' CLI symlink (default: ~/.local/bin).
      --no-cli             Do not create a CLI symlink.
  -u, --uninstall          Remove Ogun.app from destination and clean up CLI symlink.
  -h, --help               Display this help message and exit.

Examples:
  ./packaging/macos/install-app.sh
  ./packaging/macos/install-app.sh --build
  ./packaging/macos/install-app.sh build_release/src/app/Ogun.app
  ./packaging/macos/install-app.sh --uninstall
EOF
}

# Parse command line options
while [[ $# -gt 0 ]]; do
    case "$1" in
        -h|--help)
            print_usage
            exit 0
            ;;
        -b|--build)
            FORCE_BUILD=true
            shift
            ;;
        -d|--dest|--destination)
            if [[ $# -lt 2 ]]; then
                echo "Error: $1 requires a directory argument." >&2
                exit 1
            fi
            DEST_DIR="$2"
            shift 2
            ;;
        --cli-dir)
            if [[ $# -lt 2 ]]; then
                echo "Error: --cli-dir requires a directory argument." >&2
                exit 1
            fi
            CLI_DIR="$2"
            shift 2
            ;;
        --no-cli)
            INSTALL_CLI=false
            shift
            ;;
        -u|--uninstall)
            UNINSTALL=true
            shift
            ;;
        -*)
            echo "Error: Unknown option '$1'." >&2
            print_usage >&2
            exit 1
            ;;
        *)
            if [[ -z "${SOURCE_BUNDLE}" ]]; then
                SOURCE_BUNDLE="$1"
            else
                echo "Error: Multiple bundle paths specified ('${SOURCE_BUNDLE}' and '$1')." >&2
                exit 1
            fi
            shift
            ;;
    esac
done

DEST_APP="${DEST_DIR}/Ogun.app"
LSREGISTER="/System/Library/Frameworks/CoreServices.framework/Frameworks/LaunchServices.framework/Support/lsregister"

needs_sudo() {
    local target_dir="${DEST_DIR}"
    while [[ ! -d "${target_dir}" && "${target_dir}" != "/" && "${target_dir}" != "." ]]; do
        target_dir="$(dirname "${target_dir}")"
    done
    if [[ ! -w "${target_dir}" ]]; then
        return 0
    fi
    if [[ -e "${DEST_APP}" && ! -w "${DEST_APP}" ]]; then
        return 0
    fi
    return 1
}

# Helper for elevated permissions
run_cmd() {
    if needs_sudo; then
        if [[ $EUID -ne 0 ]]; then
            if command -v sudo >/dev/null 2>&1; then
                sudo "$@"
                return
            else
                echo "Error: Cannot write to ${DEST_DIR} and sudo is not available." >&2
                exit 1
            fi
        fi
    fi
    "$@"
}

# Handle uninstall
if [[ "${UNINSTALL}" = true ]]; then
    echo "==> Uninstalling Ogun from ${DEST_DIR}..."
    if [[ -d "${DEST_APP}" ]]; then
        if [[ -x "${LSREGISTER}" ]]; then
            echo "Unregistering from LaunchServices..."
            "${LSREGISTER}" -u "${DEST_APP}" 2>/dev/null || true
        fi
        echo "Removing ${DEST_APP}..."
        run_cmd rm -rf "${DEST_APP}"
    else
        echo "No application found at ${DEST_APP}."
    fi

    if [[ -L "${CLI_DIR}/ogun" ]]; then
        echo "Removing CLI symlink ${CLI_DIR}/ogun..."
        rm -f "${CLI_DIR}/ogun"
    fi

    echo "Ogun has been uninstalled successfully."
    exit 0
fi

# Locate or build Ogun.app
if [[ "${FORCE_BUILD}" = true ]] || [[ -z "${SOURCE_BUNDLE}" && ! -d "${REPO_DIR}/build_release/src/app/Ogun.app" && ! -d "${REPO_DIR}/build/src/app/Ogun.app" ]]; then
    echo "==> Building Ogun (Release)..."
    cmake -B "${REPO_DIR}/build_release" -DCMAKE_BUILD_TYPE=Release
    cmake --build "${REPO_DIR}/build_release" --target ogun -j 2
fi

if [[ -z "${SOURCE_BUNDLE}" ]]; then
    if [[ -d "${REPO_DIR}/build_release/src/app/Ogun.app" ]]; then
        SOURCE_BUNDLE="${REPO_DIR}/build_release/src/app/Ogun.app"
    elif [[ -d "${REPO_DIR}/build/src/app/Ogun.app" ]]; then
        SOURCE_BUNDLE="${REPO_DIR}/build/src/app/Ogun.app"
    else
        echo "Error: Could not find Ogun.app in build_release or build directory." >&2
        echo "Please build Ogun first with: cmake --build build_release -j 2" >&2
        exit 1
    fi
fi

if [[ ! -d "${SOURCE_BUNDLE}" ]]; then
    echo "Error: Source bundle '${SOURCE_BUNDLE}' does not exist or is not a directory." >&2
    exit 1
fi

if [[ ! -f "${SOURCE_BUNDLE}/Contents/MacOS/Ogun" ]]; then
    echo "Error: '${SOURCE_BUNDLE}' does not appear to be a valid Ogun bundle (missing Contents/MacOS/Ogun)." >&2
    exit 1
fi

# Warn if Ogun is running
if pgrep -x "Ogun" >/dev/null 2>&1; then
    echo "Warning: Ogun is currently running."
    echo "Please close running instances of Ogun before installing, or restart Ogun after installation."
fi

# Ensure destination directory exists
if [[ ! -d "${DEST_DIR}" ]]; then
    echo "Creating destination directory ${DEST_DIR}..."
    run_cmd mkdir -p "${DEST_DIR}"
fi

# Install application bundle
echo "==> Installing Ogun.app to ${DEST_APP}..."
if [[ -d "${DEST_APP}" ]]; then
    echo "Replacing existing installation at ${DEST_APP}..."
    run_cmd rm -rf "${DEST_APP}"
fi

# ditto preserves code signing, resource forks, and permissions
echo "Copying bundle using ditto..."
run_cmd /usr/bin/ditto "${SOURCE_BUNDLE}" "${DEST_APP}"

# Strip quarantine attributes if present
run_cmd xattr -dr com.apple.quarantine "${DEST_APP}" 2>/dev/null || true

# Update modification time
run_cmd touch "${DEST_APP}"

# Register with LaunchServices
if [[ -x "${LSREGISTER}" ]]; then
    echo "==> Registering with LaunchServices..."
    "${LSREGISTER}" -f "${DEST_APP}" 2>/dev/null || true
fi

# CLI Symlink
if [[ "${INSTALL_CLI}" = true ]]; then
    echo "==> Setting up CLI symlink..."
    mkdir -p "${CLI_DIR}"
    ln -sf "${DEST_APP}/Contents/MacOS/Ogun" "${CLI_DIR}/ogun"
    echo "Symlink created: ${CLI_DIR}/ogun -> ${DEST_APP}/Contents/MacOS/Ogun"

    if [[ ":$PATH:" != *":${CLI_DIR}:"* ]]; then
        echo ""
        echo "Notice: '${CLI_DIR}' is not currently in your PATH."
        echo "To run 'ogun' directly from any terminal session, add this to your ~/.zshrc or ~/.bashrc:"
        echo "  export PATH=\"${CLI_DIR}:\$PATH\""
    fi
fi

echo ""
echo "================================================================="
echo "  Ogun successfully installed in ${DEST_APP}!"
echo "================================================================="
echo "You can launch Ogun by:"
echo "  1. Opening Finder > Applications > Ogun"
echo "  2. Running: open -a /Applications/Ogun.app [model.stl]"
if [[ "${INSTALL_CLI}" = true ]]; then
    echo "  3. Running CLI: ogun [model.stl]"
fi
echo "================================================================="
