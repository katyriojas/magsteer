            ================================
                  MCS Control Library
                  Linux Version Notes
                (c) 2013 by SmarAct GmbH
            ================================


REQUIREMENTS
============

This version of the MCS Control Library requires Linux for 32 bit
or 64 bit/x86 architecture.

IMPORTANT: the 32 bit libraries are installed by default.
The 64 bit versions are installed if the -x64 option is used. If 
32 bit and 64 bit libraries are installed on the same system, 
they must be installed to different installation paths or they 
will overwrite each other!


INSTALLATION
============
The library package consists of the shared library libmcscontrol,
C header files and documentation. libmcscontrol requires other
libraries which are included in the package: 
libsmaractio, libftd2xx and libftchipid.
All libraries must be installed on the target computer.

The shell script 'install_mcs.sh' installs the libraries and C
header files to a user-definable installation path. Documentation
and other files are NOT installed.
The files are installed in the sub-directories lib and include.
Call:
    install_mcs.sh          - to install under /usr
    install_mcs.sh <path>   - to pass an installation path other than /usr
    install_mcs.sh -c       - to remove previous installations
    install_mcs.sh -x64     - to install the 64 bit libraries
If you install to a system path (e.g. the default path), you 
must run install_mcs.sh with sufficient privileges, e.g.
    sudo install_mcs.sh ...
Note, that when you uninstall, all libraries are removed. If you
have other SmarAct products that need some of the other installed
libraries you should either not uninstall or uninstall the MCS
software and then reinstall the software for the other products.


SYSTEM CONFIGURATION
====================
When an MCS controller is connected to the computer or switched on,
it is possible that the ftdi_sio driver is automatically loaded
for that device. In this case the application cannot connect to
the MCS. The kernel modules 
  ftdi_sio and usbserial
must be unloaded before launching your application or blocked from
loading, e.g. by blacklisting them in /etc/modprobe.d/.

The MCS Control library needs write access to the USB port the MCS 
device is connected to. To automatically set r/w permissions when 
an MCS is connected, a udev rule can be added to /etc/udev/rules.d/

   ATTR{idVendor}=="0403", ATTR{idProduct}=="6001", MODE="666"

The rule sets the r/w permission for everyone. If this is not
acceptable, adjust the MODE argument.




