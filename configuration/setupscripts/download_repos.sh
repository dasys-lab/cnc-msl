#!/bin/bash

## Wenn es einen Fehler gibt, breche das Skript ab
set -e
source ./funcs.sh

## SSH-Key erstellen

msg "SSH Key wird erstellt, falls nicht vorhanden" 

if [ ! -f ~/.ssh/id_rsa.pub ];
then 
  ssh-keygen
fi

msg "Hinweis: Dein SSH Key muss in deinem GITHUB-Account registriert werden und dein GITHUB-Account muss für Carpe-Noctem-Cassel freigegeben werden!"

## Installiere ROS

msg "GITHUB Repositories werden heruntergeladen, falls noch nicht vorhanden"

cd ~/cnws/src
if [ ! -d ~/cnws/src/alica ];
then
  msg "Downloading ALICA (Team-Coordination-Engine)"
  git clone git@github.com:carpe-noctem-cassel/alica.git
fi

if [ ! -d ~/cnws/src/alica ];
then
  msg "Downloading ALICA Plan Designer (Modellingtool)"
  git clone git@github.com:carpe-noctem-cassel/alica-plan-designer.git
fi

if [ ! -d ~/cnws/src/alica ];
then
  msg "Downloading Supplementary Stuff (Config, FileSystem-Tools, etc.)"
  git clone git@github.com:carpe-noctem-cassel/supplementary.git
fi

if [ ! -d ~/cnws/src/alica ];
then
  msg "Downloading Middle Size League Stuff"
  git clone git@github.com:carpe-noctem-cassel/cnc-msl.git
fi

msg "GITHUB Configuration wird eingerichtet, falls noch nicht geschehen"

sudo grep -q -F "default = tracking" ~/.gitconfig || cp ~/cnws/src/cnc-msl/configuration/gitconfig ~/.gitconfig
vim ~/.gitconfig

