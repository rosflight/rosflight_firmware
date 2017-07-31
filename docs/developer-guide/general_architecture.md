# ROSflight Firmware architecture

The firmware is organized into several "modules."  Each of these modules is designed to accomplish a designated set of tasks, and there are several interactions between these modules.  Each module is implemented as a class object, which are all members of a toplevel ROSflight object.

Although we strive for complete in-code documentation, in practice, this often gets left behind for the sake of rapid development.  If you, as a potential developer, find some portion of documentation unsatisfactory, we welcome questions on the github issues page, and pull requests which improve documentation.  Several new developers have started with first improving documentation to get a handle on how things work
