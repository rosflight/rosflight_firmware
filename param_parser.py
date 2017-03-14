#!/usr/bin/env python

import re

f = open('src/param.c')
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
            params[i]['type'] = 'param'
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
out = open('param_description.md', 'w')
# Start the table
out.write("| Parameter | Description | Default Value | Min | Max |\n")
out.write("|-----------|-------------|---------------|-----|-----|\n")
for param in params:
    out.write("| ")
    out.write(param['name'])
    out.write(" | ")
    out.write(param['description'])
    out.write(" | ")
    out.write(param['default'])
    out.write(" | ")
    out.write(param['min'])
    out.write(" | ")
    out.write(param['max'])
    out.write(" |\n")

out.close()

debug = 1
