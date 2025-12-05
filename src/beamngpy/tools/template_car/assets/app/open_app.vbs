Set WshShell = CreateObject("WScript.Shell")

' Path to python
python_path = "python"

' Start the Python server script in a hidden window
WshShell.Run "cmd /c start /b " & python_path & " -m template_car", 0, False
