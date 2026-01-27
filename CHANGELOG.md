# Changelog

## [2.0.0-beta.1](https://github.com/rosflight/rosflight_firmware/compare/v2.0.0-beta...v2.0.0-beta.1) (2026-01-27)


### Features

* add mag orientation params to firmware ([#476](https://github.com/rosflight/rosflight_firmware/issues/476)) ([2da05c1](https://github.com/rosflight/rosflight_firmware/commit/2da05c1bd158fad0a7d14b69aea49414edd37827))
* change command message to vector from named force-torque values ([71ca9f6](https://github.com/rosflight/rosflight_firmware/commit/71ca9f6038b5deaa4ab58b4248c5da2c8fc55caa))

## [2.0.0-beta](https://github.com/rosflight/rosflight_firmware/compare/v1.3.1...v2.0.0-beta) (2025-12-13)


### ⚠ BREAKING CHANGES

* restructure MAVLINK messages

### Features

* add versioning ci workflow using release-please ([350a83f](https://github.com/rosflight/rosflight_firmware/commit/350a83f16bdb3ef45a42eba36237e0a59e094822))
* added COLCON_IGNORE file -- needed for anyone who builds rosflight_ros_pkgs ([5201014](https://github.com/rosflight/rosflight_firmware/commit/52010149400867df1481c58eff3ecf40374c4a16))
* removed gnss_full from mavlink and comm manager ([4eb5247](https://github.com/rosflight/rosflight_firmware/commit/4eb52478a45fe6c4a4a2e1ba3a274de61d60fcc4))
* save mixer params to memory when canned mixer is selected ([4163d34](https://github.com/rosflight/rosflight_firmware/commit/4163d347fc99283e3596952ead7de99d14f7af75))


### Bug Fixes

* initialize batt voltage mult to 1 to avoid divide by zero ([#472](https://github.com/rosflight/rosflight_firmware/issues/472)) ([216af88](https://github.com/rosflight/rosflight_firmware/commit/216af88a26d11412019af238526f0e80b5bdbfe4))
* initialize battery voltage to max voltage ([fe5144c](https://github.com/rosflight/rosflight_firmware/commit/fe5144c7e1458161766c2c12947f15cba3c05181))
