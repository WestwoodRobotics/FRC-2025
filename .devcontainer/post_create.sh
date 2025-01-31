#!/bin/sh

echo "Post-create script starting..."

# Install OpenJDK 21
echo "Installing OpenJDK 17..."
sudo apt-get update
sudo apt-get install -y openjdk-17-jre

sudo apt-get update
sudo apt-get install -y libatomic1

# URL of the WPILib installer
URL="https://packages.wpilib.workers.dev/installer/v2025.1.1/Linux/WPILib_Linux-2025.1.1.tar.gz"

# Extract filename from URL
FILENAME=$(basename "$URL")

# Extract VERSION from filename
VERSION=$(echo "$FILENAME" | sed -e 's/WPILib_Linux-//' -e 's/\.tar\.gz//')

# Extract YEAR from VERSION
YEAR=${VERSION%%.*}

# Check if WPILib is already set up
if [ ! -d "$HOME/wpilib/$YEAR" ]; then
    echo "Downloading WPILib..."
    sudo wget "$URL"
    echo "Extracting WPILib..."
    sudo tar -xzvf "$FILENAME"
    echo "Setting up WPILib directory..."
    sudo mkdir "$YEAR"
    cd "$YEAR"
    sudo mv ../WPILib_Linux-"$VERSION"/WPILib_Linux-"$VERSION"-artifacts.tar.gz .
    echo "Extracting WPILib artifacts..."
    sudo tar xf WPILib_Linux-"$VERSION"-artifacts.tar.gz
    sudo rm WPILib_Linux-"$VERSION"-artifacts.tar.gz
    cd ..
    sudo mkdir -p ~/wpilib
    sudo mv "$YEAR" ~/wpilib
    sudo rm -rf WPILib_Linux-*
    sudo rm -rf ~/.gradle/caches/
    sudo rm -rf ~/wpilib/2025/maven/
    # Run ToolsUpdater.py
    cd ~/wpilib/"$YEAR"/tools/ && sudo python3 ToolsUpdater.py
    
    # Install VS Code extensions
    cd ~/wpilib/"$YEAR"/vsCodeExtensions && sudo find . -name "*.vsix" | sudo xargs -I {} code --install-extension {}
else
    echo "WPILib is already set up"
fi
