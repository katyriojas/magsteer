#!/bin/sh
# install script for MCSControl libraries and C header files
#
# usage:
# install_mcs.sh [-c] [path]
# -c:   library versions found under the installation base directory
#       are uninstalled
# path: the base directory for installing, e.g. /usr or /opt. 
#       defaults to /usr if omitted.

showhelp()
{
  printf "Usage: $0 [-c] [path]\n \
-c\tlibrary versions under the installation base directory\n \
\tare uninstalled\n \
path\tthe base directory for installing, e.g. /usr or /opt.\n \
\tdefaults to /usr if omitted.\n" >&2
}


uninstall()
{
  echo "uninstalling..."
  rm -f "$IPATH"/lib/libsmaractio*
  rm -f "$IPATH"/lib/libmcscontrol*
  rm -f "$IPATH"/lib/libftd2xx*
  rm -f "$IPATH"/lib/libftchipid*
  rm -f "$IPATH"/include/MCSControl.h
}


install()
{
  echo "installing $ARCH bit libraries to $IPATH"
  
  if [ ! -d "$IPATH/lib" ]; then
    mkdir -p "$IPATH/lib"
  fi
  if [ ! -d "$IPATH/include" ]; then
    mkdir -p "$IPATH/include"
  fi
  cp -a -f $SRCPATH/include/*.h "$IPATH/include/"
    
  cp -a -f $SRCPATH/lib$ARCH/libsmaractio.so* "$IPATH/lib/"
  cp -a -f $SRCPATH/lib$ARCH/libmcscontrol.so* "$IPATH/lib/"
  cp -a -f $SRCPATH/lib$ARCH/libftd2xx.so* "$IPATH/lib/"
  cp -a -f $SRCPATH/lib$ARCH/libftchipid.so* "$IPATH/lib/"
  ldconfig -n "$IPATH/lib"

  cd $OLDP
}


self="${0#./}"
base="${self%/*}"

SRCPATH=""
if [ "$base" = "$self" ]; then
    SRCPATH="$(pwd)"
else
    SRCPATH="$(pwd)/$base"
fi ;


DOUNINSTALL=""
IPATH="/usr"

if [ "$1" = "-h" ]; then
  showhelp;
  exit 2
fi

if [ "$1" = "-c" ]; then
  DOUNINSTALL=1;
  shift 1
fi

ARCH=32
if [ "$1" = "-x64" ]; then
  ARCH=64
  shift 1
fi

if [ -n "$1" ]; then
  IPATH=$1
fi

if [ ! -d "$IPATH" ]; then
  echo "installation path does not exist: $IPATH"
  return 1
fi

if [ -n "$DOUNINSTALL" ]; then
  uninstall;
else
  install;
fi



