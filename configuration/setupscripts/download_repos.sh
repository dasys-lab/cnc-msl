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
# TODO: Ask if already done, when not open github page


msg "GITHUB Repositories werden heruntergeladen, falls noch nicht vorhanden"


ghurl='git@github.com:carpe-noctem-cassel/'
# Repos to clone
repos='alica alica-plan-designer supplementary cnc-msl'

## Ordnerstruktur erstellen, falls nicht vorhanden
mkdir -p ~/cnws/src
cd ~/cnws/src

for r in $repos
do
	if [ ! -d $r ]
	then
		msg "Cloning repository $r"
		git clone $ghurl$r'.git'
	else
		msg "$r already exists!"
	fi
done


