#!/bin/bash

## Wenn es einen Fehler gibt, breche das Skript ab
set -e
source ./funcs.sh

## SSH-Key erstellen

msg "SSH Key wird erstellt, falls nicht vorhanden"

if [ ! -f ~/.ssh/id_rsa.pub ]; then
    msg "SSH Key erstellen ..."
    ssh-keygen -b 4096
    msg "SSH Key wurde erstellt"
else
    msg "SSH Key vorhanden"
fi

msg "Hinweis: Dein SSH Key muss in deinem GITHUB-Account registriert werden und dein GITHUB-Account muss für Carpe-Noctem-Cassel freigegeben werden!"
if askSure "Auf GITHUB gehen?"; then
  xdg-open "https://github.com/carpe-noctem-cassel"
fi

msg "GITHUB Repositories werden heruntergeladen, falls noch nicht vorhanden"

ghurl='git@github.com:carpe-noctem-cassel/'
# Repos to clone
repos='alica alica-plan-designer supplementary cnc-msl msl_gazebo_simulator'

## Ordnerstruktur erstellen, falls nicht vorhanden
mkdir -p ~/cnws/src
cd ~/cnws/src

for r in $repos; do
    if [ ! -d $r ]; then
        msg "Cloning repository $r"
        git clone --depth 1 $ghurl$r'.git'
    else
        msg "$r already exists!"
    fi
done


# Git einrichten
## gitconfig in Benutzerverzeichniss kopieren wenn noch nicht vorhanden oder übername gewünscht ist
if [ ! -f ~/.gitconfig ] || askSure "Gitconfig von CarpeNoctemCassel  übernehmen? (Bestehende ~/.gitconfig wird überschrieben!)"; then
    cp ~/cnws/src/cnc-msl/configuration/gitconfig ~/.gitconfig
fi

## Benutzernamen und Email abfragen und eintragen
if askSure "GIT-E-Mail und -Name einrichten?"; then
    getInput "E-Mail Adresse für GIT eingeben" "email"
    sed -i "/email/c\\\temail = ${email}" ~/.gitconfig
    msg "E-Mail Adresse gesetzt"

    getInput "Vollen Namen für GIT eingeben" "name"
    sed -i "/name/c\\\tname = ${name}" ~/.gitconfig
    msg "Benutzername gesetzt"
fi
