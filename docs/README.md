## Build documentation preview

You can build the documentation locally for preview:

Install dependencies:

```
pip install -r docs/source/requirements.txt
```

Build docs:

```
cd docs
sphinx-build -M html source build
```

Open `docs/build/html/index.html` in a web browser.
