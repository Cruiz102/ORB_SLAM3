#!/bin/bash

echo "========================================="
echo "VS Code IntelliSense Setup Checker"
echo "========================================="
echo ""

# Colors
GREEN='\033[0;32m'
RED='\033[0;31m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

# Check if we're in the right directory
if [ ! -f "CMakeLists.txt" ]; then
    echo -e "${RED}❌ Error: Run this from ORB_SLAM3 root directory${NC}"
    exit 1
fi

echo "Checking VS Code configuration files..."
echo ""

# Check .vscode directory
if [ -d ".vscode" ]; then
    echo -e "${GREEN}✅ .vscode directory exists${NC}"
else
    echo -e "${RED}❌ .vscode directory NOT found${NC}"
fi

# Check individual files
files=(
    ".vscode/c_cpp_properties.json:C++ IntelliSense configuration"
    ".vscode/settings.json:Workspace settings"
    ".vscode/tasks.json:Build tasks"
    ".vscode/launch.json:Debug configuration"
    ".vscode/extensions.json:Extension recommendations"
    "build/compile_commands.json:Compile commands database"
)

for item in "${files[@]}"; do
    IFS=':' read -r file desc <<< "$item"
    if [ -f "$file" ]; then
        size=$(du -h "$file" | cut -f1)
        echo -e "${GREEN}✅${NC} $desc ($size)"
    else
        echo -e "${RED}❌${NC} $desc - NOT FOUND"
    fi
done

echo ""
echo "Checking system dependencies..."
echo ""

# Check for required tools
commands=(
    "g++:C++ compiler"
    "cmake:CMake build system"
    "gdb:GNU Debugger"
    "pkg-config:Package config tool"
)

for item in "${commands[@]}"; do
    IFS=':' read -r cmd desc <<< "$item"
    if command -v "$cmd" &> /dev/null; then
        version=$($cmd --version 2>&1 | head -n1)
        echo -e "${GREEN}✅${NC} $desc: $version"
    else
        echo -e "${YELLOW}⚠️${NC}  $desc: NOT FOUND"
    fi
done

echo ""
echo "Checking libraries..."
echo ""

# Check for libraries using pkg-config
libs=("opencv4" "eigen3")

for lib in "${libs[@]}"; do
    if pkg-config --exists "$lib" 2>/dev/null; then
        version=$(pkg-config --modversion "$lib")
        echo -e "${GREEN}✅${NC} $lib: v$version"
    else
        echo -e "${YELLOW}⚠️${NC}  $lib: NOT FOUND via pkg-config"
    fi
done

# Check for Pangolin (harder to detect)
if [ -d "/usr/local/include/pangolin" ] || [ -d "/usr/include/pangolin" ]; then
    echo -e "${GREEN}✅${NC} Pangolin: Found"
else
    echo -e "${YELLOW}⚠️${NC}  Pangolin: Headers not found in standard locations"
fi

echo ""
echo "Checking ORB_SLAM3 build..."
echo ""

if [ -f "lib/libORB_SLAM3.so" ]; then
    size=$(du -h "lib/libORB_SLAM3.so" | cut -f1)
    echo -e "${GREEN}✅${NC} ORB_SLAM3 library: $size"
else
    echo -e "${RED}❌${NC} ORB_SLAM3 library not built - run ./build.sh first"
fi

if [ -f "Examples/Monocular/mono_tum" ]; then
    size=$(du -h "Examples/Monocular/mono_tum" | cut -f1)
    echo -e "${GREEN}✅${NC} mono_tum executable: $size"
else
    echo -e "${YELLOW}⚠️${NC}  mono_tum not built"
fi

echo ""
echo "========================================="
echo "Summary"
echo "========================================="
echo ""

# Count results
total_config_files=6
found_config_files=$(ls .vscode/*.json build/compile_commands.json 2>/dev/null | wc -l)

if [ "$found_config_files" -eq "$total_config_files" ]; then
    echo -e "${GREEN}✅ All configuration files present${NC}"
else
    echo -e "${YELLOW}⚠️  Some configuration files missing ($found_config_files/$total_config_files)${NC}"
fi

echo ""
echo "Next steps:"
echo "1. Open this folder in VS Code"
echo "2. Install recommended extensions (VS Code will prompt)"
echo "3. Reload window (Ctrl+Shift+P → Reload Window)"
echo "4. Open Examples/Monocular/mono_tum.cc and test IntelliSense"
echo ""
echo "For detailed help, see: .vscode/QUICKSTART.md"
echo ""
