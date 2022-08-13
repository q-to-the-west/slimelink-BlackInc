# SlimeLink

Connect devices with nDoF sensors to SlimeVR.

## Daydream Instructions

- Tested on *Python 3.10*
- You must have `quaternion`, `numpy`, `bitstring` and `bleak` libraries
  - Use this [quaternion](https://quaternion.readthedocs.io/en/latest/) library
- If core libraries are missing, check the source file for the requirements
- Change IP address and port to the *SlimeVR* server if needed in `server` variable
- Run `python Daydream.py`
- Turn on your *Daydream* controllers by pressing a *Circle* button (second in the middle)
- You can only place your controllers facing upwards (touchpad area at the top)
  - You can place them by either having side with buttons at the front or at the back
- Setup positions in *SlimeVR*
  - If rotation is inverted, change position in the software to the opposite side
- *Windows* works worse than *Linux* (30 TPS on *Windows* vs 60 TPS on *Linux*)
  - Tested on *openSUSE Tumbleweed*

## Disclaimer

The project is a mess, Windows doesn't work and I don't know when I can continue the development. Sorry.
