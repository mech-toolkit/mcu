#!/bin/bash

MAJOR=$(grep "#define VERSION_MAJOR" version.h | awk '{print $3}')
MINOR=$(grep "#define VERSION_MINOR" version.h | awk '{print $3}')
PATCH=$(grep "#define VERSION_PATCH" version.h | awk '{print $3}')

echo "Current MECH Version: $MAJOR.$MINOR.$PATCH"
echo "Which version component do you want to increment? (1) Major (2) Minor (3) Patch"
read -p "> " choice

case $choice in
  1)
    MAJOR=$((MAJOR+1))
    MINOR=0
    PATCH=0
    ;;
  2)
    MINOR=$((MINOR+1))
    PATCH=0
    ;;
  3)
    PATCH=$((PATCH+1))
    ;;
  *)
    echo "Invalid choice."
    exit 1
    ;;
esac

sed -i "s/#define VERSION_MAJOR [0-9]\+/#define VERSION_MAJOR $MAJOR/g" version.h
sed -i "s/#define VERSION_MINOR [0-9]\+/#define VERSION_MINOR $MINOR/g" version.h
sed -i "s/#define VERSION_PATCH [0-9]\+/#define VERSION_PATCH $PATCH/g" version.h

echo "New version: $MAJOR.$MINOR.$PATCH"
