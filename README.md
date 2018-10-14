# UART Driver

Simple UART Driver that explores:
- Memory mapped io
- `Platform` Drivers (UART Unit of BBB)
- `Misc` Drivers (Character device `/dev/serial-481a8000`)
- `ioctl` Syscalls

The driver is part of my experimental _BeagleBone Black_ [buildroot project].

It was initially based on bootlin's [Linux kernel and driver development training].



[//]:  #  (Reference Links)
[Linux kernel and driver development training]: <https://bootlin.com/doc/training/linux-kernel/linux-kernel-labs.pdf>

[buildroot project]: <https://bitbucket.org/MarcoHartmann/buildroot_bbb/src>