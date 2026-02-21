#!/bin/bash
# Wrapper script to run the C++ tracker server

set -e  # Exit on error

# Configuration
BUILD_DIR="server/build"
EXECUTABLE="$BUILD_DIR/bin/tracker_server"

# Default arguments
ESTIMATOR="imm"
MOTOR="mock"
CAMERA="0"
VIZ_FLAG=""  # Empty by default, visualization is ON by default in C++

# Parse command line arguments or use defaults
while [[ $# -gt 0 ]]; do
    case $1 in
        --estimator)
            ESTIMATOR="$2"
            shift 2
            ;;
        --motor)
            MOTOR="$2"
            shift 2
            ;;
        --camera)
            CAMERA="$2"
            shift 2
            ;;
        --no-display)
            VIZ_FLAG="--no-viz"
            shift
            ;;
        --help)
            echo "Usage: $0 [OPTIONS]"
            echo ""
            echo "Options:"
            echo "  --estimator TYPE    Estimator type: kalman_cv, kalman_ca, or imm (default: imm)"
            echo "  --motor TYPE        Motor controller: mock or simplefoc (default: mock)"
            echo "  --camera ID         Camera device ID (default: 0)"
            echo "  --no-display        Disable video display window"
            echo "  --help              Show this help message"
            echo ""
            echo "Examples:"
            echo "  $0                                    # Run with defaults (IMM estimator, mock motor, camera 0)"
            echo "  $0 --estimator kalman_cv              # Use constant velocity Kalman filter"
            echo "  $0 --motor simplefoc                  # Use SimpleFOC motor controller"
            echo "  $0 --camera 1 --no-display            # Use camera 1 without display"
            exit 0
            ;;
        *)
            echo "Unknown option: $1"
            echo "Run '$0 --help' for usage information"
            exit 1
            ;;
    esac
done

# Check if executable exists
if [ ! -f "$EXECUTABLE" ]; then
    echo "Executable not found at $EXECUTABLE"
    echo "Building project..."
    echo ""
    
    # Create build directory if it doesn't exist
    mkdir -p "$BUILD_DIR"
    
    # Run CMake configuration
    cd "$BUILD_DIR"
    cmake ..
    
    # Build the project
    make -j$(nproc)
    
    # Return to root directory
    cd ../..
    
    echo ""
    echo "Build complete!"
    echo ""
fi

# Run the tracker
echo "Starting tracker with:"
echo "  Estimator: $ESTIMATOR"
echo "  Motor: $MOTOR"
echo "  Camera: $CAMERA"
echo "  Display: $([ -z "$VIZ_FLAG" ] && echo "enabled" || echo "disabled")"
echo ""

cd "$BUILD_DIR"

# Configure Qt for display
# Use XWayland compatibility layer for broader compatibility
export QT_QPA_PLATFORM=xcb
export QT_LOGGING_RULES='*.debug=false;qt.qpa.*=false;qt.gui.font*=false'
export QT_DEBUG_PLUGINS=0

# Ensure X11 display is available (XWayland)
export DISPLAY="${DISPLAY:-:0}"

# Avoid snap library conflicts that can cause version mismatches
unset GIO_MODULE_DIR

./bin/tracker_server --estimator "$ESTIMATOR" --motor "$MOTOR" --camera "$CAMERA" $VIZ_FLAG
