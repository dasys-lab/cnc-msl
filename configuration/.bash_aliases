# Start aliases
alias LEBT='rosrun LebtClient LebtClient.exe'
alias PROXY='rosrun MSLDDSProxy udpProxy'
alias CAN='rosrun CNCanProxy CNUsbCanConsole'
alias PLAN='rosrun Planmodeller start.sh'

# Connect to robots
alias hairy='ssh -X cn@hairy'
alias savvy='ssh -X cn@savvy'
alias nase='ssh -X cn@nase'
alias myo='ssh -X cn@myo'
alias mops='ssh -X cn@mops'
alias debug='ssh -X cn@debug'

# Shutdown and reboot of robots
alias hairydown='ssh cn@hairy -t sudo shutdown now -h'
alias hairyreboot='ssh cn@hairy -t sudo reboot'
alias savvydown='ssh cn@savvy -t sudo shutdown now -h'
alias savvyreboot='ssh cn@savvy -t sudo reboot'
alias nasedown='ssh cn@nase -t sudo shutdown now -h'
alias nasereboot='ssh cn@nase -t sudo reboot'
alias myodown='ssh cn@myo -t sudo shutdown now -h'
alias myoreboot='ssh cn@myo -t sudo reboot'
alias mopsdown='ssh cn@mops -t sudo shutdown now -h'
alias mopsreboot='ssh cn@mops -t sudo reboot'
alias debugdown='ssh cn@debug -t sudo shutdown now -h'
alias debugreboot='ssh cn@debug -t sudo reboot'

