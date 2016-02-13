#!/bin/bash

## Wenn es einen Fehler gibt, breche das Skript ab
set -e
source ./funcs.sh

## System gegebenenfalls updaten
if askSure "Paketquellen aktualisieren?"
then
	msg "Ubuntu Paketquellen werden aktualisiert"
	sudo apt-get "${1}" update
	if askSure "Pakete aktualisieren?"
	then
		msg "Ubuntu Pakete werden bei bedarf geupdatet"
		sudo apt-get "${1}" upgrade
	fi
fi

## Installiere allgemeine Pakete fuer Entwicklung
msg "Allgemeine Pakete zur Entwicklung werden installiert"

packages='git vim gitk meld bison re2c libode-dev gnuplot-qt
	libxv-dev libtbb-dev libcgal-demo libcgal-dev
	xsdcxx libxerces-c-dev freeglut3-dev libvtk5-dev libvtk5-qt4-dev'


echo $packages
eval sudo apt-get "${1}" install $packages

