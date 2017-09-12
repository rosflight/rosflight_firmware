#!/usr/bin/env python
#
__author__ = "James Jackson"
__copyright__ = "Copyright 2017, ROSflight"
__license__ = "BSD-3"
__version__ = "0.1"
__maintainer__ = "James Jackson"
__email__ = "superjax08@gmail.com"

import re

f = open('src/param.cpp')
text = f.read()


lines = re.split("\n+", text)

params = []
i = 0
for line in lines:
    # search for init_param lines
    match = re.search("^\s*init_param", line)
    if match != None:
        name_with_quotes = re.search("\".*\"", line).group(0)
        name = re.search("\w{1,16}", name_with_quotes).group(0)
        if name != 'DEFAULT':
            params.append(dict())
            params[i]['type'] = re.split("\(", re.split("_", line)[2])[0]
            params[i]['name'] = name
            # Find default value
            params[i]['default'] =  re.split("\)", re.split(",", line)[2])[0]
            # isolate the comment portion of the line and split it on the "|" characters
            comment = re.split("\|", re.split("//", line)[1])
            # Isolate the Description
            params[i]["description"] = comment[0].strip()
            params[i]["min"] = comment[1].strip()
            params[i]["max"] = comment[2].strip()
            i += 1

# Now, generate the markdown table of the parameters
out = open('parameter-descriptions.md', 'w')
# Print the title
out.write("# Parameter descriptions\n\n")
# Start the table
out.write("| Parameter | Description | Type | Default Value | Min | Max |\n")
out.write("|-----------|-------------|------|---------------|-----|-----|\n")
for param in params:
    out.write("| ")
    out.write(param['name'])
    out.write(" | ")
    out.write(param['description'])
    out.write(" | ")
    out.write(param['type'])
    out.write(" | ")
    out.write(param['default'])
    out.write(" | ")
    out.write(param['min'])
    out.write(" | ")
    out.write(param['max'])
    out.write(" |\n")

out.close()

debug = 1
