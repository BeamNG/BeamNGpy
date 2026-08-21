# Script to run tests on CI
$ErrorActionPreference = 'Stop'
$env:BNG_HOME = $env:BNG_HOME_WINDOWS

# PowerShell does not fail on non-zero exit codes of native commands
function Invoke-Checked {
    $command, $arguments = $args
    & $command @arguments
    if ($LASTEXITCODE -ne 0) {
        throw "'$args' failed with exit code $LASTEXITCODE."
    }
}

Write-Host "Environment variables:"
Write-Host "TESTING_REVISION: $env:TESTING_REVISION"
Write-Host "BNG_HOME: $env:BNG_HOME"
Write-Host "HEADLESS: $env:HEADLESS"
Write-Host "GFX: $env:GFX"

Write-Host "Setting up Python environment ..."
Remove-Item -Recurse -Force .venv -ErrorAction SilentlyContinue
Invoke-Checked python -m venv .venv
. .\.venv\Scripts\Activate.ps1
Invoke-Checked pip install -e .
Invoke-Checked pip install pytest

Write-Host "Cleaning up BeamNG user folder ..."
Remove-Item -Recurse -Force "$env:LOCALAPPDATA\BeamNG" -ErrorAction SilentlyContinue

Write-Host "Running tests ..."
Invoke-Checked pytest tests/test_minimal.py
