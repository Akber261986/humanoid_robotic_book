#!/bin/bash
# Startup script for Railway deployment

# Install dependencies
pip install -r requirements.txt

# Start the application
python run_server.py