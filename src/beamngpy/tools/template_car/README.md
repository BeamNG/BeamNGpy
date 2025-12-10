# Template Car Generator

This module contains the source code of the Template Car Generator.

We use [FastAPI](https://fastapi.tiangolo.com/) to serve a REST API and plain HTML/CSS/JavaScript for the UI, which is rendered with [pywebview](https://pypi.org/project/pywebview/).

Run with:
```
python -m beamngpy.tools.template_car
```

## Developing

Run Template Car Generator server in development mode:

```
cd beamngpy
fastapi dev src/beamngpy/tools/template_car/api.py
```

Then you can:
* [Open web app in your browser](http://localhost:8000)
* [View API docs](http://localhost:8000/docs)
* [View API redoc](http://localhost:8000/redoc)

Directory structure:

```sh
src/beamngpy/tools/template_car/
├── assets/          # static assets included in the Python package
│   ├── web/         # contains the HTML/CSS/JavaScript for the UI
│   ├── template/    # contains the template JBeam and DAE files for the vehicle generation
│   ├── app/         # helper files for the desktop shortcut
├── *.py files       # Python source code
├── README.md        # this file
```
