#!/bin/bash

# UTM System - Quick Start Script
# This script helps you launch the entire system easily

echo "╔════════════════════════════════════════════════════════════╗"
echo "║     UTM SYSTEM - Unmanned Traffic Management              ║"
echo "║           Quick Start Launcher                            ║"
echo "╚════════════════════════════════════════════════════════════╝"
echo ""

# Check Python version
echo "🔍 Checking Python installation..."
if ! command -v python3 &> /dev/null; then
    echo "❌ Python 3 is not installed. Please install Python 3.10 or higher."
    exit 1
fi

PYTHON_VERSION=$(python3 --version 2>&1 | awk '{print $2}')
echo "✓ Python $PYTHON_VERSION found"
echo ""

# Install dependencies
echo "📦 Installing dependencies..."
pip install -r requirements.txt --quiet
echo "✓ Dependencies installed"
echo ""

# Launch options
echo "Select launch mode:"
echo "  1) Full System (Backend + Simulator)"
echo "  2) Backend Only"
echo "  3) Simulator Only"
echo ""
read -p "Enter choice [1-3]: " choice

case $choice in
    1)
        echo ""
        echo "🚀 Launching Full UTM System..."
        echo ""
        echo "Starting backend in background..."
        python3 main.py &
        BACKEND_PID=$!
        
        sleep 5
        
        echo "Starting virtual drone fleet..."
        python3 drone_simulator.py &
        SIMULATOR_PID=$!
        
        echo ""
        echo "════════════════════════════════════════════════════════"
        echo "✓ System is running!"
        echo ""
        echo "📡 Backend API:    http://localhost:8000"
        echo "🌐 Frontend UI:    http://localhost:8000"
        echo "📚 API Docs:       http://localhost:8000/docs"
        echo ""
        echo "Backend PID:       $BACKEND_PID"
        echo "Simulator PID:     $SIMULATOR_PID"
        echo ""
        echo "Press Ctrl+C to stop all services..."
        echo "════════════════════════════════════════════════════════"
        
        # Wait for interrupt
        trap "echo ''; echo 'Stopping services...'; kill $BACKEND_PID $SIMULATOR_PID 2>/dev/null; exit 0" INT
        wait
        ;;
    
    2)
        echo ""
        echo "🚀 Launching Backend Only..."
        python3 main.py
        ;;
    
    3)
        echo ""
        echo "🚀 Launching Simulator Only..."
        echo "(Make sure backend is running at http://localhost:8000)"
        sleep 2
        python3 drone_simulator.py
        ;;
    
    *)
        echo "Invalid choice. Exiting."
        exit 1
        ;;
esac
