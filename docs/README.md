## Build documentation preview

You can build the documentation locally for preview:

Install dependencies:

```
pip install -r docs/source/requirements.txt
```

### Current version

Build currently checked out version of docs:

```
cd docs
sphinx-build -M html source build
```

Open `docs/build/html/index.html` in a web browser.

### All versions

Build all versions:

```
cd docs
sphinx-multiversion source build
```

Note: Some older version tags will be ignored because the directory structure of the `docs` directory changed at some point.
`sphinx-multiversion` requires at least `docs/source` and `docs/source/conf.py` to exist in order to build a version.
