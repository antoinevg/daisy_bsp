# Changelog
All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.0.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

<!--
## [Unreleased]
-->

## [0.4.2] - 2021-08-01
### Added
- Expose the (Q)SPI pins to access flash memory (Tx @phoracek!)

---

## [0.4.1] - 2021-04-27
### Added
- New feature: `alloc`

### Changed
- Dropped the Peripheral Access Crate implementation of the audio driver in favour of the `embedded_hal` implementation.
- Refactored the crate to make the requirement for [rust nightly and the `alloc` crate](https://doc.rust-lang.org/edition-guide/rust-next/alloc.html) optional. See the [`audio_*` examples](examples/) for more details.
- Refactored the crate to remove implicit definitions of `#[interrupt]` handlers in favour of explicit declaration in program code. See the [`audio_*` examples](examples/) for more details.

---

## [0.3.2] - 2021-04-01
### Changed
- Changed license to MIT

## [0.3.0] - 2021-03-24
### Added
- New feature: `log-itm`
- New example: `itm`

### Changed
- Refactored the structure of the crate to avoid the problem of the Board interface taking ownership of pac/hal singletons. Check out the `examples/` directory for more information.
- Modified the MIDI API to work with the Daisy Pod's MIDI pin numbers by default.
- Updated `stm32h7xx_hal` to `0.9.0`

---

## [0.2.0] - 2021-02-03
### Added
- New examples: `adc_hal`, `adc_bsp`
- New member for `daisy::Board`: `board.peripheral`
- Set peripheral clock to: `4 MHz`


## [0.1.0] - 2020-11-28
### Added
- Initial release
- Supported features are:
  * Default clock configuration @ 480 MHz
  * User LED
  * Audio
  * MIDI (via USART1)


[Unreleased]: https://github.com/antoinevg/daisy_bsp/compare/v0.3.0...HEAD
[0.2.0]: https://github.com/antoinevg/daisy_bsp/compare/v0.1.0...v0.2.0
[0.1.0]: https://github.com/antoinevg/daisy_bsp/releases/tag/v0.1.0
