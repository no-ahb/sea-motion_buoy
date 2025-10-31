# Git Repository Setup

This document explains how to use Git for tracking changes to the project.

## Current Status

✅ Git repository initialized
✅ Initial commit created with project structure
✅ Files organized and tracked

## Next Steps: GitHub Setup

To push this repository to GitHub:

1. **Create a GitHub repository:**
   - Go to https://github.com/new
   - Choose a name (e.g., `ocean-motion-buoy`, `sea-motion-sensor`)
   - Don't initialize with README (we already have one)

2. **Add remote and push:**
   ```bash
   git remote add origin https://github.com/YOUR_USERNAME/YOUR_REPO_NAME.git
   git branch -M main  # Rename master to main if desired
   git push -u origin main
   ```

## Submodule Note

The BNO085 library is detected as a submodule. You can either:

**Option A: Keep as submodule** (recommended if you want to track library updates):
```bash
git submodule init
git submodule update
```

**Option B: Remove submodule tracking** (if you want to include library files directly):
```bash
git rm --cached scripts/library/BOSCH-BNO085-I2C-micropython-library
rm -rf scripts/library/BOSCH-BNO085-I2C-micropython-library/.git
git add scripts/library/BOSCH-BNO085-I2C-micropython-library
git commit -m "Include BNO085 library files directly"
```

## Workflow Tips

- **Commit frequently:** Small, focused commits are easier to track
- **Use TODO.txt:** Agents can update this file to track changes
- **Update CHANGELOG.md:** Document version changes to scripts
- **Organize recordings:** Use consistent naming and folder structure

## Binary Files

Binary recordings (`.bin`) are currently tracked in Git. If they become too large:

1. Use Git LFS: `git lfs track "*.bin"`
2. Or add to `.gitignore` and use external storage

