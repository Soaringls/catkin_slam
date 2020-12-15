#!/usr/bin/env bash
#By dexter @ 2020/07/24

set -e 
# set -x

FILE_DIR=$(dirname $(readlink -f "$0"))
echo "file dir:$FILE_DIR"

if [ $# -lt 1 ];then
  echo "usage:bash <script> r[reset] | d4[docker-gcc4.8] | x[xps]"
  exit
fi

#reset devel-environment
if [ $1 == r ];then
  echo "reset the development environment..."
  cd $FILE_DIR/../
  #reset opt
  if [ -e opt ];then
    if [ ! -e opt.xps.new ];then
      echo "reset opt to opt.xps.new"
      mv opt opt.xps.new
    fi
    if [ ! -e opt.d4.new ];then
      echo "reset opt to opt.d4.new"
      mv opt opt.d4.new
    fi
  fi
  # if [ ! -e opt.d8 ];then
  #   echo "reset opt to opt.d8"
  #   mv opt opt.d8
  # fi
  #reset gears
  cd $FILE_DIR/../gears
  if [ -e x86_64 ];then
    if [ ! -e x86_64.d4.new ];then
      echo "reset gears/x86_64 to x86_64.d4.new"
      mv x86_64 x86_64.d4.new
    fi
    # if [ ! -e x86_64.d8 ];then
    #   echo "reset gears/x86_64 to x86_64.d8"
    #   mv x86_64 x86_64.d8
    # fi
    if [ ! -e x86_64.xps.new ];then
      echo "reset gears/x86_64 to x86_64.xps.new"
      mv x86_64 x86_64.xps.new
    fi
  fi
fi

#convert to docker-gcc4.8
cd $FILE_DIR/..
if [ $1 == d4 ];then
  echo "convert to docker-gcc4.8 envionment..."
  if [ ! -e opt ];then
    mv opt.d4.new opt
  fi
  cd $FILE_DIR/../gears
  if [ ! -e x86_64 ];then
    mv x86_64.d4.new x86_64
  fi
fi
#convert to xps
cd $FILE_DIR/..
if [ $1 == x ];then
  echo "convert to xps envionment..."
  if [ ! -e opt ];then
    mv opt.xps.new opt
  fi
  cd $FILE_DIR/../gears
  if [ ! -e x86_64 ];then
    mv x86_64.xps.new x86_64
  fi
fi