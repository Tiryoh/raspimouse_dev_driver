# raspimouse_dev_driver

[![Build Status](https://travis-ci.org/Tiryoh/raspimouse_dev_driver.svg?branch=master)](https://travis-ci.org/Tiryoh/raspimouse_dev_driver)

device driver of Raspberry Pi Mouse

## Requirements

* Raspberry Pi 2 or Raspberry Pi 3
  * tested only on Raspberry Pi 3

## Installation

First, check `build-essentials` has been installed.

```
sudo apt install build-essential
```

Next, download this repository.

```
git clone https://github.com/Tiryoh/raspimouse_dev_driver.git
```

After that, move into `raspimouse_dev_driver/src` directory and run make command.

```
cd raspimouse_dev_driver/src
make && sudo make install
```

If you need to uninstall this driver, run make command with uninstall argument.

```
sudo make uninstall
```

## Usage

After installation, test gpio check scripts.

The LED on the board will blink.

```
./scripts/test_leds.sh
```

Status of switches will shown.

```
./scripts/test_switches.sh
```


## References & Includings

This device driver is derived from this repository.

* [rt-net/RaspberryPiMouse](https://github.com/rt-net/RaspberryPiMouse)
* [Tiryoh/raspigibbon_driver](https://github.com/Tiryoh/raspigibbon_driver)

## License

This repository is licensed under the GPLv3 license, see [LICENSE](./LICENSE).

