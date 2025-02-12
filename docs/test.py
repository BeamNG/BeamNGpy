import os

content = """
 <html>
 <head>
   <title>beamngpy</title>
 </head>
 <body>
   <h1>Hello from beamngpy!</h1>
 </body>
 </html>
"""

output_dir = "build"
output_file = f"{output_dir}/index.html"

if not os.path.isdir(output_dir):
    os.makedirs(output_dir)

with open(output_file, 'w') as f:
    f.write(content)
