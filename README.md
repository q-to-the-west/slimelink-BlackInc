# SlimeLink

Connect devices with nDoF sensors (Google Daydream controllers) to SlimeVR.

## Daydream Instructions

- Tested on *Python 3.12.4* on *Windows*.
- External dependencies:
  - `quaternion`,
    - Use this [quaternion](https://quaternion.readthedocs.io/en/latest/) library.
  - `numpy`,
  - `bitstring` and
  - `bleak` libraries.
  - If core libraries are missing, check the source file for the requirements.
- Open SlimeVR Server.
- Place your controllers facing upward (touchpad area at the top) and forward (touchpad area in front).
- Run `python Daydream.py`
- Turn on your *Daydream* controllers by pressing the *Circle* button (located at middle of the controller, beneath the — button).
- Set up positions in *SlimeVR* and continue to use the controllers as full-body tracking devices.

## Disclaimer

This project is still unstable; more reliable communications between this code and SlimeVR's firmware protocol are still in progress.