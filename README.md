# DK2 Transform Filter

This is a Transform Filter that implements the unpacking of erroneously-declared YUY2 frames. It lets you use the DK2 positional tracking camera as a regular video source on Windows by wiring up a [DirectShow Filter Graph](http://msdn.microsoft.com/en-us/library/windows/desktop/dd407188%28v=vs.85%29.aspx). This is the first step for getting opensource positional tracking working on Windows [OpenHMD](http://openhmd.net/). More detail (and a Linux version of the same) can be found on the [Hacking the Oculus Rift DK2](http://doc-ok.org/?p=1095) series from Doc Ok.

Build and register the filter .dll using a tool like [GraphEditPlus](http://www.infognition.com/GraphEditPlus/) which can also generate boilerplate code for you.

![GraphPlus Screenshot](https://raw.githubusercontent.com/wiki/anarchist/dk2-transform-filter/images/dk2-graph.png)
