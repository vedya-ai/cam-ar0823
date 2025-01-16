# Changelog

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.0.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [1.1.0] - 2024-07-11

### Added
- Cropped 1600x1280 @ 30FPS mode.
- Documentation on frame synchronization modes.

### Changed
- Frame synchronization is now controlled via a custom v4l2 control.
- Updated GMSL datatype filtering to be per serializer video pipe.

## [1.0.2] - 2024-04-08

### Added
- Support for multiple sensor modes and mode tables.
- Custom sensor modes for external pulse-based frame sync and shutter trigger-based frame sync.

### Removed
- Debug Devicetree template for Sony AR0823 FPD-Link modules.

### Fixed
- Sensor now enters start-up/standby mode when probed, rather than staying in streaming mode.
- Power on, power off, start and stop streaming functions streamlined.
- Null pointer dereference on GMSL platforms.

## [1.0.1] - 2024-03-06

### Added
- Support for Sony AR0823 FPD-Link module.
- Automated Debian packaging.
- Support for lane polarity swizzle in carrier boards.

### Fixed
- Fixed missing source file specified in Kbuild.

### Changed
- Updated build system to support GMSL and FPD-Link variants.
- Use debhelper compatibility version 13 instead of 9.
- Use common d3-hardware.dtsi defines from carrier boards.
- Removed standard includes from DTS build system.

## [1.0.0] - 2023-07-21

Initial release of D3's AR0823 camera module with support for FPD-Link variants.

### Added
- Support for D3 AR0823 FPD-Link module.

[1.0.2]: https://gitlab.d3engineering.com/nvidia/lkms/d3-module-ar0823/-/compare/D3%2FRELEASE%2F1.0.2...D3%2FRELEASE%2F1.1.0
[1.0.2]: https://gitlab.d3engineering.com/nvidia/lkms/d3-module-ar0823/-/compare/D3%2FRELEASE%2F1.0.1...D3%2FRELEASE%2F1.0.2
[1.0.1]: https://gitlab.d3engineering.com/nvidia/lkms/d3-module-ar0823/-/compare/D3%2FRELEASE%2F1.0.0...D3%2FRELEASE%2F1.0.1
[1.0.0]: https://gitlab.d3engineering.com/nvidia/lkms/d3-module-ar0823/-/tags/D3%2FRELEASE%2F1.0.0
