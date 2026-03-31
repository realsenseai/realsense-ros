#!/bin/bash

# Copyright 2026 RealSense, Inc. All Rights Reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

# Validates that package.xml dependency declarations are complete.
# Catches issues the build farm would catch but rosdep+apt misses:
#   - .action files require action_msgs in build_depend
#   - find_package() calls need matching build_depend/depend entries
#   - rosidl_generate_interfaces DEPENDENCIES need matching entries
#
# Usage: ./check_package_deps.sh <workspace_src_dir>

SRC_DIR="${1:-.}"
EXIT_CODE=0

error() {
    echo "ERROR: $1" >&2
    EXIT_CODE=1
}

for pkg_xml in $(find "$SRC_DIR" -name "package.xml" -not -path "*/build/*" -not -path "*/install/*"); do
    pkg_dir=$(dirname "$pkg_xml")
    pkg_name=$(sed -n 's/.*<name>\([^<]*\)<\/name>.*/\1/p' "$pkg_xml")
    cmake_file="$pkg_dir/CMakeLists.txt"

    [ -f "$cmake_file" ] || continue

    echo "Checking $pkg_name..."

    # Extract declared build dependencies from package.xml
    declared_deps=$(sed -n 's/.*<build_depend>\([^<]*\)<\/build_depend>.*/\1/p; s/.*<depend>\([^<]*\)<\/depend>.*/\1/p' "$pkg_xml")

    # Check 1: .action files require action_msgs
    if grep -q '\.action"' "$cmake_file" 2>/dev/null; then
        if ! echo "$declared_deps" | grep -qx "action_msgs"; then
            error "$pkg_name: has .action files but missing <build_depend>action_msgs</build_depend>"
        fi
    fi

    # Check 2: find_package() calls should have matching package.xml entries
    # Uses rosdep to distinguish ROS packages (must be declared) from system libraries (skip)
    find_pkgs=$(sed -n 's/.*find_package(\s*\([A-Za-z0-9_-]*\).*/\1/p' "$cmake_file" | sort -u)
    for dep in $find_pkgs; do
        # Skip buildtool dependencies (declared as <buildtool_depend>)
        case "$dep" in
            ament_cmake*|rosidl_default_generators) continue ;;
        esac
        # Skip packages not known to rosdep (system/CMake libraries like OpenCV, Qt5, Eigen3)
        if ! rosdep resolve "$dep" >/dev/null 2>&1; then
            continue
        fi
        if ! echo "$declared_deps" | grep -qx "$dep"; then
            error "$pkg_name: find_package($dep) but missing <build_depend>$dep</build_depend> in package.xml"
        fi
    done

    # Check 3: rosidl_generate_interfaces DEPENDENCIES should be declared
    # Extract everything between DEPENDENCIES and the next keyword/closing paren
    rosidl_deps=$(sed -n '/DEPENDENCIES/,/)/p' "$cmake_file" | sed 's/.*DEPENDENCIES//' | tr ')' '\n' | head -1 | tr ' ' '\n' | grep '^[a-z]' | grep -v '^add_linter' || true)
    for dep in $rosidl_deps; do
        if ! echo "$declared_deps" | grep -qx "$dep"; then
            error "$pkg_name: rosidl DEPENDENCIES lists '$dep' but missing <build_depend>$dep</build_depend> in package.xml"
        fi
    done
done

if [ $EXIT_CODE -ne 0 ]; then
    echo ""
    echo "Dependency check FAILED. Fix the above errors to ensure build farm compatibility."
    exit 1
fi

echo "All dependency checks passed."
