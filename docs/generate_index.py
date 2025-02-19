import sys

content = """
<!DOCTYPE html>
<html>
  <head>
    <title>Redirecting</title>
    <meta charset="utf-8">
    <meta http-equiv="refresh" content="0; url=./{BRANCH}/index.html">
    <link rel="canonical" href="./{BRANCH}/index.html">
  </head>
</html>
"""

def main():
    branch = sys.argv[1]
    output_file = sys.argv[2]
    s = generate_redirect(branch)
    with open(output_file, 'w') as f:
        f.write(s)


def generate_redirect(branch):
    return content.replace("{BRANCH}", branch)


if __name__ == "__main__":
    main()
