# Writing Documentation

All documentation for the entire ROSflight stack (both firmware and ROS code) belongs in this "ReadTheDocs" webpage.  Here I will explain quickly how to build the documentation on your own local computer so you can view changes as you write and contribute to the documentation.

## Install mkdocs and latex support

This is easy:

``` bash
sudo apt install mkdocs
sudo pip3 install https://github.com/mitya57/python-markdown-math/archive/master.zip
```

(You don't have use to the global pip if you have python environments working, but for beginners, this is the simplest way to do it)

## Run the mkdocs server

Just type

``` bash
mkdocs serve
```

in the root directory of the firmware repository.  It should report to you something like

``` bash
[I 170728 07:49:47 server:271] Serving on http://127.0.0.1:8000
[I 170728 07:49:47 handlers:58] Start watching changes
```

This means that mkdocs is hosting a webpage for you on http://127.0.0.1:8000.  Navigate to that page in your web browser.

Now, as you make changes to the documentation, you should be able to see it on your browser.  Just hit reload from time to time to see your changes.

## Adding pages
To add a new page to the documentation, just check out the mkdocs.yaml file in the root of the firmware directory, you should be able to figure it out from there.

## Adding latex
The syntax for adding latex math inline is `\( x \)` which renders as \(x \).  For adding a block, it's

``` latex
$$ E = mc^2 $$
```
which renders as
$$ E = mc^2 $$
