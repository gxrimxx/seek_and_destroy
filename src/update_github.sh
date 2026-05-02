#!/bin/bash

# 1. Copy the latest ROSject notebook into your Git workspace
echo "Grabbing the latest Jupyter Notebook..."
cp ~/notebook_ws/default.ipynb ~/seek_destroy_ws/src

# 2. Add all changes to Git (including the newly copied notebook)
git add .

# 3. Commit with the message you provide when running the script
git commit -m "$1"

# 4. Push to GitHub
echo "Pushing to GitHub..."
git push -u origin master --force