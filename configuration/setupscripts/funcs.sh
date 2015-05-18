#!/bin/bash

function msg {
    echo -e "\e[0;32m> $1 \e[0m"
}

# This function adds a line to bashrc if its not already present
# It's a modified version of: http://stackoverflow.com/questions/3557037/appending-a-line-to-a-file-only-if-it-doesnt-already-exist-using-sed
function add_to_bashrc {
    grep -q -F "$1" ~/.bashrc || echo "$1" >> ~/.bashrc
}
