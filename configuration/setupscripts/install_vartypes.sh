#!/bin/bash

## Wenn es einen Fehler gibt, breche das Skript ab
set -e
source ./funcs.sh

## Installiere vartypes

msg "VARTYPES wird heruntergeladen und installiert, falls noch nicht geschehen"

if [ ! -d /usr/local/include/vartypes ];
then
  cd ~
  wget http://vartypes.googlecode.com/files/vartypes-0.7.tar.gz
  tar xfz vartypes-0.7.tar.gz
  cd vartypes-0.7
  mkdir build && cd build
  cmake ..
  make 
  sudo make install
  cd ..
  rm -rf vartypes-07.tar.gz
  rm -rf vartypes-0.7
fi
