#!/usr/bin/env python3
"""
Direct fix for pygazebo - modifies the file before any imports
"""

import os
import sys

# Find the pygazebo file directly
pygazebo_file = "/usr/local/lib/python3.10/dist-packages/pygazebo-3.0.0_2014.1-py3.10.egg/pygazebo/pygazebo.py"

if not os.path.exists(pygazebo_file):
    print(f"Error: File not found at {pygazebo_file}")
    sys.exit(1)

print(f"Fixing {pygazebo_file}...")

# Read the file
with open(pygazebo_file, 'r') as f:
    content = f.read()

# Apply fixes
# Fix asyncio.async which is the problematic syntax
content = content.replace('asyncio.async(', 'asyncio.ensure_future(')

# Write back
with open(pygazebo_file, 'w') as f:
    f.write(content)

print("Fix applied!")

# Now test the import
try:
    import pygazebo
    print("Import successful!")
except Exception as e:
    print(f"Import still failed: {e}")
    sys.exit(1)