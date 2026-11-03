#!/usr/bin/env bash
# Registers Ogun desktop launcher and .stl MIME association for the current user.
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_DIR="$(cd "${SCRIPT_DIR}/../.." && pwd)"

APPS_DIR="${HOME}/.local/share/applications"
MIME_DIR="${HOME}/.local/share/mime/packages"
ICON_DIR="${HOME}/.local/share/icons/hicolor/512x512/apps"
BIN_DIR="${HOME}/.local/bin"

mkdir -p "${APPS_DIR}" "${MIME_DIR}" "${ICON_DIR}" "${BIN_DIR}"

# Link or copy binary
BINARY_PATH="${1:-${REPO_DIR}/build_release/src/app/ogun}"
if [[ -f "${BINARY_PATH}" ]]; then
    echo "Installing binary symlink to ${BIN_DIR}/ogun..."
    ln -sf "${BINARY_PATH}" "${BIN_DIR}/ogun"
fi

# Copy icon
echo "Installing application icon to ${ICON_DIR}/ogun.png..."
cp "${REPO_DIR}/assets/icons/ogun.png" "${ICON_DIR}/ogun.png"

# Copy MIME file
echo "Installing MIME type to ${MIME_DIR}/ogun.xml..."
cp "${SCRIPT_DIR}/ogun-mime.xml" "${MIME_DIR}/ogun.xml"

# Copy desktop file
echo "Installing desktop entry to ${APPS_DIR}/ogun.desktop..."
cp "${SCRIPT_DIR}/ogun.desktop" "${APPS_DIR}/ogun.desktop"

# Update databases
if command -v update-mime-database >/dev/null 2>&1; then
    update-mime-database "${HOME}/.local/share/mime"
fi

if command -v update-desktop-database >/dev/null 2>&1; then
    update-desktop-database "${APPS_DIR}"
fi

if command -v xdg-mime >/dev/null 2>&1; then
    xdg-mime default ogun.desktop model/stl
    xdg-mime default ogun.desktop application/sla
fi

echo "Ogun desktop integration installed successfully!"
