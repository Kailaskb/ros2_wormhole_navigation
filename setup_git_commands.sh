#!/bin/bash

# Rename the default branch from master to main
git branch -m master main

# Add all files to staging
git add .

# Create initial commit
git commit -m "Initial commit: Multi-Map Navigation System with VDA 5050 support"

# Instructions for pushing to GitHub or other remote
echo "==================================================="
echo "Repository initialized with branch 'main'"
echo "To push to GitHub or another Git hosting service:"
echo ""
echo "1. Create a new repository at GitHub/GitLab/etc."
echo "2. Connect your local repo to the remote:"
echo "   git remote add origin https://github.com/yourusername/ros2_vda5050_multi_map_nav.git"
echo ""
echo "3. Push your code to the remote repository:"
echo "   git push -u origin main"
echo "==================================================="
