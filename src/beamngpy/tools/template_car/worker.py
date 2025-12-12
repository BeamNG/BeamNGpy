import sys
from .core import TemplateCarGenerator


def main():
    vehicle_id = sys.argv[1]
    settings_file = sys.argv[2]
    generator = TemplateCarGenerator(settings_file=settings_file)
    vehicle = generator.get_vehicle(vehicle_id)
    try:
        generator.generate(vehicle_id, vehicle)
    except Exception as e:
        print(error_string(str(e)))

def error_string(text: str):
    return f"\n--- ERROR ---\n{text}"


if __name__ == "__main__":
    main()
