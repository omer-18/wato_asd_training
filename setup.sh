#!/bin/bash
# Quick setup script for WATonomous ASD Training Assignment

echo "=========================================="
echo "WATonomous ASD Training - Setup Script"
echo "=========================================="
echo ""

# Check if Docker is running
echo "1. Checking Docker..."
if ! docker ps > /dev/null 2>&1; then
    echo "   ERROR: Docker is not running. Please start Docker Desktop or Docker Engine."
    exit 1
fi
echo "   ✓ Docker is running"
echo ""

# Check watod-config.sh
echo "2. Checking configuration..."
if grep -q 'ACTIVE_MODULES="robot gazebo vis_tools"' watod-config.sh; then
    echo "   ✓ Active modules are correctly set"
else
    echo "   WARNING: Active modules may not be set correctly in watod-config.sh"
fi
echo ""

# Build modules
echo "3. Building module images (this may take a while)..."
./watod build
if [ $? -eq 0 ]; then
    echo "   ✓ Build completed successfully"
else
    echo "   ERROR: Build failed"
    exit 1
fi
echo ""

# Setup dev environment
echo "4. Setting up development environment for Intellisense..."
./watod --setup-dev-env robot
echo "   ✓ Dev environment setup complete"
echo ""

echo "=========================================="
echo "Setup complete!"
echo "=========================================="
echo ""
echo "Next steps:"
echo "1. Run './watod up' to start the containers"
echo "2. Check the logs for the Foxglove URL (should look like https://localhost:#####)"
echo "3. Open Foxglove and connect to that URL"
echo "4. Import the layout from config/wato_asd_training_foxglove_config.json"
echo ""

