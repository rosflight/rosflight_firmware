# Contributing To the Firmware

Per our vision stated in the [introduction](/), ROSflight is intended to be a streamlined, bare-bones autopilot.  We welcome any bug fixes, cleanup, or other contributions which do not add complexity or detract from the readability and simplified nature of the firmware.  We hope that the firmware is useful, but in an attempt to avoid "feature creep" we will be very discriminatory in merging pull requests whose purpose is to simply add features.  Forking the repository in order to add features is totally acceptable and encouraged, just stay in contact with us and recognize us as the original authors of the autopilot (per the agreement in the BSD-3 license).

In addition, we strive to maintain a very high standard of quality in terms of code style, variable naming, and the like.  We will likely be nit-picky and perhaps a little harsh about this in pull requests.  Please do not be offended.  By maintaining a high standard, we hope that the code will continue to be useful, understandable, and cohesive in nature.

Pull requests are also required to pass the automated unit tests. You can test your changes against these unit tests before pushing by executing the `run_tests.sh` script in the root directory of the firmware repo.

Although we strive for complete in-code documentation, in practice this often gets left behind for the sake of rapid development.  If you, as a potential developer, find some portion of documentation unsatisfactory, we welcome questions on the [GitHub issues page](https://github.com/rosflight/firmware/issues) or [forum](https://discuss.rosflight.org/), and pull requests which improve documentation.  Several new developers have started with first improving documentation to get a handle on how things work.

# Communication

There are two channels to communicate with the developer team.  For bug reports, feature requests, and anything to do with code, please open an issue on the appropriate [firmware](https://github.com/rosflight/firmware/issues) or [ROS stack](https://github.com/rosflight/rosflight/issues) GitHub issue page.  For questions and other discussions, please use the [forum](https://discuss.rosflight.org/).
