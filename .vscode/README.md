# VS Code IntelliSense Setup for ORB-SLAM3

This directory contains VS Code configuration files to enable proper C++ IntelliSense (code completion, error detection, go-to-definition, etc.) for the ORB-SLAM3 project.

## Configuration Files Created

- **`.vscode/c_cpp_properties.json`** - C++ IntelliSense configuration with include paths
- **`.vscode/settings.json`** - VS Code workspace settings
- **`.vscode/extensions.json`** - Recommended extensions

## Required VS Code Extensions

Install these extensions (VS Code will prompt you to install them):

1. **C/C++** (`ms-vscode.cpptools`) - Microsoft's C++ extension for IntelliSense
2. **CMake Tools** (`ms-vscode.cmake-tools`) - CMake support
3. **CMake** (`twxs.cmake`) - CMake language support

## Setup Steps

### 1. Install Extensions

Open VS Code command palette (Ctrl+Shift+P) and run:
```
Extensions: Show Recommended Extensions
```

Then install all recommended extensions.

### 2. Generate compile_commands.json

The `compile_commands.json` file tells IntelliSense exactly how each file is compiled. It's already configured to be generated automatically when you build.

For the main project:
```bash
cd /home/cesar/Projects/ORB_SLAM3
./build.sh
```

For the mono_tum example:
```bash
cd Examples/Monocular
./build.sh
```

### 3. Reload VS Code

After building, reload VS Code:
- Press `Ctrl+Shift+P`
- Type "Reload Window"
- Press Enter

## Verifying It Works

After setup, you should have:

✅ **Syntax highlighting** - Code is properly colored  
✅ **Error detection** - Red squiggles under actual errors  
✅ **Auto-completion** - Suggestions when typing (Ctrl+Space)  
✅ **Go to definition** - F12 or Ctrl+Click on symbols  
✅ **Hover information** - Hover over variables/functions to see types  
✅ **Find all references** - Shift+F12 to find where symbols are used  

## Troubleshooting

### IntelliSense not working

1. **Check compile_commands.json exists:**
   ```bash
   ls -lh build/compile_commands.json
   ```

2. **Reload the IntelliSense database:**
   - Open Command Palette (Ctrl+Shift+P)
   - Run: `C/C++: Reset IntelliSense Database`

3. **Check the selected configuration:**
   - Bottom right of VS Code should show "Linux"
   - Click it to change if needed

4. **Check Output panel:**
   - View → Output
   - Select "C/C++" from dropdown
   - Look for errors

### Include paths not found

If you see errors like `#include <opencv2/core/core.hpp> not found`:

1. Find where the library is installed:
   ```bash
   # For OpenCV
   pkg-config --cflags opencv4
   
   # For Eigen
   pkg-config --cflags eigen3
   ```

2. Update `.vscode/c_cpp_properties.json` with the correct paths

### Slow IntelliSense

If IntelliSense is slow or freezes:

1. Close unnecessary files
2. Add exclusions in settings:
   ```json
   "C_Cpp.exclusionPolicy": "checkFolders",
   "files.watcherExclude": {
       "**/build/**": true,
       "**/Vocabulary/**": true
   }
   ```

## Project Structure

The IntelliSense is configured to understand:

- **Main includes:** `/home/cesar/Projects/ORB_SLAM3/include`
- **Camera models:** `/include/CameraModels`
- **Third-party libraries:**
  - Sophus (in `Thirdparty/Sophus`)
  - DBoW2 (in `Thirdparty/DBoW2`)
  - g2o (in `Thirdparty/g2o`)
- **System libraries:**
  - Eigen3 (`/usr/include/eigen3`)
  - OpenCV (`/usr/include/opencv4`)
  - Pangolin (`/usr/local/include/pangolin`)

## Advanced: Per-File Configuration

VS Code IntelliSense uses `compile_commands.json` which contains the exact compilation command for each file. This means:

- Each `.cc` file gets its specific include paths and defines
- No manual configuration needed per file
- Automatically updated when you rebuild

## Additional Resources

- [VS Code C++ Documentation](https://code.visualstudio.com/docs/languages/cpp)
- [C/C++ Extension Guide](https://code.visualstudio.com/docs/cpp/config-linux)
- [CMake Tools Documentation](https://github.com/microsoft/vscode-cmake-tools)
