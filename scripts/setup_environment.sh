#!/bin/bash
# setup_environment.sh - Configures environment variables

echo "[1/2] Setting up environment variables..."
echo 'export PATH=$HOME/.local/bin:$PATH' >> ~/.bashrc
echo 'export LD_LIBRARY_PATH=$HOME/.local/lib:$LD_LIBRARY_PATH' >> ~/.bashrc

echo "[2/2] Sourcing bashrc..."
source ~/.bashrc
echo "Environment configured successfully"
