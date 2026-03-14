#!/usr/bin/env python3
"""
Fix pygazebo compatibility with Python 3.10+
This script must run before importing pygazebo
"""

import os
import sys
import glob

def fix_pygazebo():
    """Fix asyncio.async syntax error in pygazebo for Python 3.10+"""

    # Find pygazebo installation
    possible_paths = [
        "/usr/local/lib/python*/dist-packages/pygazebo*/pygazebo/pygazebo.py",
        "/usr/local/lib/python*/site-packages/pygazebo*/pygazebo/pygazebo.py",
        "/usr/lib/python*/dist-packages/pygazebo/pygazebo.py",
        "/usr/lib/python*/site-packages/pygazebo/pygazebo.py",
    ]

    pygazebo_file = None
    for pattern in possible_paths:
        matches = glob.glob(pattern)
        if matches:
            pygazebo_file = matches[0]
            break

    if not pygazebo_file:
        print("Warning: pygazebo not found, skipping fix")
        return False

    print(f"Fixing pygazebo at: {pygazebo_file}")

    try:
        # Read the file
        with open(pygazebo_file, 'r') as f:
            content = f.read()

        # Check if fix is needed
        if 'asyncio.async(' in content:
            # Apply fix
            content = content.replace('asyncio.async(', 'asyncio.ensure_future(')

            # Write back
            with open(pygazebo_file, 'w') as f:
                f.write(content)

            print("pygazebo fix applied successfully")
        else:
            print("pygazebo already fixed or doesn't need fixing")

        return True

    except Exception as e:
        print(f"Error applying fix: {e}")
        return False

if __name__ == "__main__":
    success = fix_pygazebo()
    sys.exit(0 if success else 1)