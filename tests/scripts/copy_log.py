"""Copy the BeamNG log file to the current directory."""
import os
import shutil


def main():
    if os.name == "nt":
        src_file = os.path.join(os.environ["LOCALAPPDATA"], "BeamNG", "BeamNG.tech", "current", "beamng.log")
    else:
        src_file = os.path.expanduser("~/.local/share/BeamNG/BeamNG.tech/current/beamng.log")
    dst_file = "beamng.log"
    print(f"Copying file {src_file} to {dst_file}")
    try:
        shutil.copy(src_file, dst_file)
    except FileNotFoundError:
        print(f"File {src_file} not found")
    except Exception as e:
        print(f"Error copying file {src_file} to {dst_file}: {e}")


if __name__ == "__main__":
    main()
