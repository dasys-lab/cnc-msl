#!/bin/bash

function msg {
    echo -e "\e[0;32m> $1 \e[0m"
}

# This function adds a line to bashrc if its not already present
# It's a modified version of: http://stackoverflow.com/questions/3557037/appending-a-line-to-a-file-only-if-it-doesnt-already-exist-using-sed
# $1: line to add
# $2: file in which the line should be add
# $3: if set to "as_root" (selfexplaining && optional)
function add_to
{
    if [ "${3}" == "as_root" ]; then
        grep -q -F "$1" "$2" || sudo sh -c "echo $1 >> $2"
    else
        grep -q -F "$1" "$2" || echo "$1" >> "$2"
    fi
}

# This function asks after $1 if it is accepted it returns 0 else 1
# Used in conditions and loops
function askSure
{
    # Ask question
    echo -n "${1} (Y/N)"
    # Read answer
    while read -r -n 1 -s answer
    do
        if [[ "${answer}" = [YyNn] ]]
        then
            [[ "${answer}" = [Yy] ]] && retval=0
            [[ "${answer}" = [Nn] ]] && retval=1
            break
        fi
    done

    echo # just a final linefeed, optics

    return "${retval}"
}

# Read console input with additional check if the input is correct
# $1: Text to ask for the input
# $2: Variable name in which the input should be stored
function getInput
{
    while read -p "${1}: " input
    do
        if askSure "Bist du dir sicher mit deiner Eingabe?"
        then
            eval "$2=\${input}"
            break
        fi
    done
}

