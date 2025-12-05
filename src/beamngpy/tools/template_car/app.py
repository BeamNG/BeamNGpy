"""Application with graphical user interface."""

import subprocess
import webview
import argparse
import os
from .shortcut import create_desktop_shortcut

def app_main():
    # CLI arguments
    module_name = 'beamngpy.tools.template_car'
    parser = argparse.ArgumentParser(description='Template Car Generator', prog=f'python -m {module_name}')
    parser.add_argument('--port', type=int, default=18052, help='Port to run the web server on')
    parser.add_argument('--settings-file', type=str, default='', help='Use custom path to a settings JSON file')
    parser.add_argument('--create-shortcut',action='store_true', help='Create a Desktop shortcut to this app and exit')
    args = parser.parse_args()
    if args.create_shortcut:
        create_desktop_shortcut()
        print("Desktop shortcut created.")
        return

    # Start server
    app_class = f"{module_name}.api:app"
    env = os.environ.copy()
    if args.settings_file != '':
        env['TEMPLATE_CAR_SETTINGS_FILE'] = args.settings_file
    proc = subprocess.Popen(["uvicorn", app_class, "--port", f"{args.port}"], env=env)

    # Open user interface
    options = dict(
        width=1400,
        height=1300,
        text_select=True,
        zoomable=True,
        min_size=(700, 500)
    )
    webview.create_window('Template Car Generator', f'http://localhost:{args.port}', **options)
    webview.start()

    # When window is closed, terminate the server
    proc.terminate()
