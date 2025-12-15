import os
import tempfile
import subprocess
import platform
from .core import TemplateCarGenerator
from .utils import assets_dir


def create_desktop_shortcut():
    """Create a desktop shortcut for the Template Car Generator."""
    desktop_path = os.path.join(os.path.expanduser("~"), "Desktop")
    generator = TemplateCarGenerator()
    icon_file = os.path.join(generator.install_path, "icon-beamng.ico")
    description = "Launch BeamNG Template Car Generator"
    working_dir = os.getcwd()

    if platform.system() == "Windows":
        target = os.path.abspath(os.path.join(assets_dir(), "app", "open_app.vbs"))
        shortcut_file = os.path.join(desktop_path, "BeamNG Template Car Generator.lnk")
        create_shortcut_windows(target, shortcut_file, working_dir, icon_file, description)
    else:
        applications_path = os.path.expanduser("~/.local/share/applications")
        shortcut_file = os.path.join(applications_path, "BeamNG Template Car Generator.desktop")
        icon_file = "BeamNG.tech.png"  # requires that BeamNG_install_desktop_file.sh has been run
        create_shortcut_linux(shortcut_file, working_dir, icon_file, description)


def create_shortcut_windows(target, shortcut_file, working_dir, icon_file, description=""):
    vbs_code = f'''
    Set oWS = WScript.CreateObject("WScript.Shell")
    sLinkFile = "{shortcut_file}"
    Set oLink = oWS.CreateShortcut(sLinkFile)
    oLink.TargetPath = "{target}"
    oLink.WorkingDirectory = "{working_dir}"
    oLink.Description = "{description}"
    oLink.IconLocation = "{icon_file}"
    oLink.Save
    '''
    with tempfile.NamedTemporaryFile('w', suffix='.vbs', delete=False) as f:
        f.write(vbs_code)
        vbs_path = f.name

    subprocess.run(["wscript", vbs_path], check=True)
    os.remove(vbs_path)


def create_shortcut_linux(shortcut_file, working_dir, icon_file, description):
    s = f"""[Desktop Entry]
Name=BeamNG Template Car Generator
Exec=python3 -m beamngpy.tools.template_car
Type=Application
Terminal=false
Comment={description}
Path={working_dir}
Icon=${icon_file}
"""
    with open(shortcut_file, 'w') as f:
        f.write(s)
    os.chmod(shortcut_file, 0o755)
