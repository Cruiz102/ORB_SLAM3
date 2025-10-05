# Quick Start Guide - VS Code IntelliSense for ORB-SLAM3

## ✅ What's Been Set Up

Your VS Code workspace is now configured with:

1. **C++ IntelliSense configuration** (`.vscode/c_cpp_properties.json`)
2. **Workspace settings** (`.vscode/settings.json`)
3. **Build tasks** (`.vscode/tasks.json`)
4. **Debug configuration** (`.vscode/launch.json`)
5. **Extension recommendations** (`.vscode/extensions.json`)
6. **Compile commands generated** (`build/compile_commands.json`)

## 🚀 Quick Steps to Get IntelliSense Working

### 1. Install Required Extensions

When you open this project in VS Code, you'll see a notification asking to install recommended extensions. Click **"Install All"**.

Or manually install:
- C/C++ (Microsoft)
- CMake Tools (Microsoft)
- CMake (twxs)

### 2. Reload VS Code

Press `Ctrl+Shift+P` → Type "Reload Window" → Press Enter

### 3. Test It!

Open `Examples/Monocular/mono_tum.cc` and:

- **Hover** over `cv::Mat` - you should see its definition
- **Ctrl+Click** on `ORB_SLAM3::System` - it should jump to System.h
- **Type** `SLAM.` - you should see auto-complete suggestions
- Press `F12` on any function - it should go to definition

## 🔧 Building with VS Code

### Option 1: Use Terminal (Recommended)
```bash
cd Examples/Monocular
./build.sh
```

### Option 2: Use VS Code Tasks
- Press `Ctrl+Shift+B` (build default task)
- Or press `Ctrl+Shift+P` → "Tasks: Run Task" → Select "build mono_tum"

## 🐛 Debugging

1. Set breakpoints by clicking left of line numbers (red dots)
2. Press `F5` to start debugging
3. **Note:** Update the dataset path in `.vscode/launch.json` first:
   ```json
   "args": [
       "${workspaceFolder}/Vocabulary/ORBvoc.txt",
       "${workspaceFolder}/Examples/Monocular/TUM1.yaml",
       "/path/to/your/dataset"  // <- Change this
   ]
   ```

## 🔍 If IntelliSense Still Doesn't Work

### Check 1: Verify compile_commands.json exists
```bash
ls -lh build/compile_commands.json
```

Should show ~136K file.

### Check 2: Check IntelliSense configuration
- Look at bottom-right corner of VS Code
- Should say "C++ ✓" or show "Linux"
- Click it if you see an error

### Check 3: Reset IntelliSense
1. Press `Ctrl+Shift+P`
2. Type "C/C++: Reset IntelliSense Database"
3. Press Enter
4. Wait a few seconds

### Check 4: View IntelliSense logs
1. View → Output (or `Ctrl+Shift+U`)
2. Select "C/C++" from dropdown
3. Look for errors

## 📝 Common Issues & Solutions

### "Cannot find opencv2/core/core.hpp"

IntelliSense might not find system headers. Update include paths:
```bash
# Find OpenCV includes
pkg-config --cflags opencv4

# Add the path to .vscode/c_cpp_properties.json
```

### IntelliSense is slow

Add to `.vscode/settings.json`:
```json
"files.watcherExclude": {
    "**/build/**": true,
    "**/Vocabulary/**": true,
    "**/.git/**": true
}
```

### Red squiggles everywhere but code compiles fine

1. Make sure you built with Debug mode (already set in build.sh)
2. Reload IntelliSense database (Ctrl+Shift+P → Reset IntelliSense)
3. Check Output panel for errors

## 🎯 Keyboard Shortcuts

- `F12` - Go to Definition
- `Shift+F12` - Find All References  
- `Ctrl+Space` - Trigger Auto-Complete
- `Ctrl+.` - Quick Fix
- `F2` - Rename Symbol
- `Ctrl+Shift+B` - Build
- `F5` - Start Debugging
- `Ctrl+K Ctrl+I` - Show Hover Info

## 📚 More Help

See `.vscode/README.md` for detailed documentation.

## ✨ You're All Set!

Your VS Code should now have full C++ IntelliSense support for ORB-SLAM3. Happy coding! 🎉
