$VENV_DIRECTORY = ".venv"

# Create the virtual environment if it doesn't already exist
if (-Not $(Test-Path $VENV_DIRECTORY)) {
    Write-Host "Creating environment $VENV_NAME"
    python.exe -m venv $VENV_DIRECTORY
}

# Activate environment
.\.venv\Scripts\activate.ps1

Write-Host "Installing dependencies from requirements.txt $VENV_NAME"
# Install python dependencies
pip install -r requirements.txt 

# Keep window open
Pause