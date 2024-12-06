#!/bin/bash

# Ensure Homebrew is installed
if ! command -v brew &>/dev/null; then
    echo "Homebrew not found. Installing Homebrew..."
    /bin/bash -c "$(curl -fsSL https://raw.githubusercontent.com/Homebrew/install/HEAD/install.sh)"
fi

# Update Homebrew
brew update

# Install Python 3
brew install python

# Install PyGObject and GStreamer dependencies
brew install pygobject3 gstreamer gst-plugins-base gst-plugins-good gst-plugins-bad gst-plugins-ugly gst-libav

# Install PyQt5
brew install pyqt

# Install OpenCV for Python
brew install opencv
pip3 install opencv-python

# Verify installation
echo "Verifying installation..."

if python3 -c "import gi; gi.require_version('Gst', '1.0'); gi.require_version('GstApp', '1.0'); from gi.repository import Gst" &>/dev/null; then
    echo "GStreamer modules are installed successfully."
else
    echo "Error: GStreamer modules installation failed."
fi

if python3 -c "from PyQt5.QtCore import QObject, pyqtSignal" &>/dev/null; then
    echo "PyQt5 is installed successfully."
else
    echo "Error: PyQt5 installation failed."
fi

if python3 -c "import cv2" &>/dev/null; then
    echo "OpenCV is installed successfully."
else
    echo "Error: OpenCV installation failed."
fi

echo "All dependencies are installed."
