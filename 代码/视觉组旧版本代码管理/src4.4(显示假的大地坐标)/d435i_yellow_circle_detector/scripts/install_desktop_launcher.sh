#!/usr/bin/env bash
set -e

PACKAGE_DIR="$HOME/uav_ws/src/d435i_yellow_circle_detector"
START_SCRIPT="$PACKAGE_DIR/scripts/start_d435i_record.sh"
LAUNCHER_NAME="D435i_YellowCircle_Record.desktop"

if [ ! -f "$START_SCRIPT" ]; then
  echo "ERROR: Start script not found: $START_SCRIPT"
  echo "Please make sure the package is located at:"
  echo "  $PACKAGE_DIR"
  exit 1
fi

chmod +x "$START_SCRIPT"

write_launcher() {
  local desktop_dir="$1"
  local desktop_file="$desktop_dir/$LAUNCHER_NAME"

  mkdir -p "$desktop_dir"

  cat > "$desktop_file" <<EOF
[Desktop Entry]
Type=Application
Name=D435i YellowCircle Record
Comment=Run D435i yellow circle detector and save image dataset
Exec=bash -lc "bash $START_SCRIPT"
Icon=camera-photo
Terminal=true
Categories=Development;Science;
StartupNotify=false
EOF

  chmod +x "$desktop_file"

  if command -v gio >/dev/null 2>&1; then
    gio set "$desktop_file" metadata::trusted true >/dev/null 2>&1 || true
  fi

  echo "  $desktop_file"
}

DESKTOP_DIRS=()

XDG_DESKTOP_DIR="$(xdg-user-dir DESKTOP 2>/dev/null || true)"
if [ -n "$XDG_DESKTOP_DIR" ] && [ "$XDG_DESKTOP_DIR" != "$HOME" ]; then
  DESKTOP_DIRS+=("$XDG_DESKTOP_DIR")
fi

DESKTOP_DIRS+=("$HOME/Desktop")

UNIQUE_DIRS=()
for dir in "${DESKTOP_DIRS[@]}"; do
  skip=false
  for existing in "${UNIQUE_DIRS[@]}"; do
    if [ "$existing" = "$dir" ]; then
      skip=true
      break
    fi
  done
  if [ "$skip" = false ]; then
    UNIQUE_DIRS+=("$dir")
  fi
done

echo "Desktop launcher created in:"
for dir in "${UNIQUE_DIRS[@]}"; do
  write_launcher "$dir"
done
echo
echo "If Ubuntu shows 'Untrusted application launcher', right-click the icon and choose 'Allow Launching'."
echo "If the icon is still not visible on the desktop, open the directory printed above in Files."
