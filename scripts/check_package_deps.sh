#!/bin/bash
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
    find_pkgs=$(sed -n 's/.*find_package(\s*\([A-Za-z0-9_-]*\).*/\1/p' "$cmake_file" | sort -u)
    for dep in $find_pkgs; do
        case "$dep" in
            ament_cmake*|rosidl_default_generators|PkgConfig|Python3|Threads|Boost|OpenCV|OpenMP|OpenGL|PCL|Eigen3|Qt5|realsense2|realsense2-gl)
                continue ;;
        esac
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
