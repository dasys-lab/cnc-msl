#!/bin/bash
export workHome="$DOMAIN_FOLDER"
updateGlobals="$DOMAIN_FOLDER/shell-scripts/./update-globals.sh"
file="$workHome/etc/Globals.conf"
switchFile="$workHome/shell/scripts/SwitchGoalTeamColour.sh"
checkBlue=`grep "OwnGoalColour = blue" $file`
checkBlueNo=`grep "#OwnGoalColour = blue" $file`
checkYellow=`grep "OwnGoalColour = yellow" $file`
checkYellowNo=`grep "#OwnGoalColour = yellow" $file`
checkCyan=`grep "OwnTeamColour = cyan" $file`
checkCyanNo=`grep "#OwnTeamColour = cyan" $file`
checkMagenta=`grep "OwnTeamColour = magenta" $file`
checkNo=`grep "#OwnTeamColour = magenta" $file`
hostName="$hostname"
#array=( zwerg mops bart fransen muecke scotti Brain hairy nase savvy myo )
array=( 172.16.40.15 )

if [ -e "$file" ]; then
  cp $workHome/etc/Globals.conf $workHome/etc/Globals.bak1

  if [ -e "$workHome/etc/Globals.bak1" ]; then

    echo ""
    echo "++++++++++++++++++ Goal colour +++++++++++++++++" 
    echo -n "Switch to blue (press b) or yellow (press y): "
    read inputGoal
    echo ""
    echo "++++++++++++++++++ Team colour ++++++++++++++++++" 
    echo -n "Switch to cyan (press c) or magenta (press m): "
    read inputTeam
    echo ""

    if [ "$inputGoal" = y ]; then
      if [ -z "$checkBlue" -o -z "$checkYellow" ]; then
        sed -e "/OwnGoalColour = blue/{;d;}" $file > $workHome/etc/Globals.bak
        mv $workHome/etc/Globals.bak $file
        sed -e "/OwnGoalColour = yellow/{N;d;}" $file > $workHome/etc/Globals.bak
        mv $workHome/etc/Globals.bak $file
        sed -i "2 i\ " $file
        sed -i "3 i\ \t#OwnGoalColour = blue" $file
        sed -i "4 i\ \tOwnGoalColour = yellow" $file
      fi
      if [ "$checkBlue" -o "$checkYellow" ]; then
        sed -e "/OwnGoalColour = blue/{;d;}" $file > $workHome/etc/Globals.bak
        mv  $workHome/etc/Globals.bak $file
        sed -e "/OwnGoalColour = yellow/{N;d;}" $file > $workHome/etc/Globals.bak
        mv  $workHome/etc/Globals.bak $file
        sed -i "2 i\ " $file
        sed -i "3 i\ \t#OwnGoalColour = blue" $file
        sed -i "4 i\ \tOwnGoalColour = yellow" $file
      fi

    elif [ "$inputGoal" = b ]; then
      if [ -z "$checkBlue" -o -z "$checkYellow" ]; then
        sed -e "/OwnGoalColour = blue/{;d;}" $file > $workHome/etc/Globals.bak
        mv $workHome/etc/Globals.bak $file
        sed -e "/OwnGoalColour = yellow/{N;d;}" $file > $workHome/etc/Globals.bak
        mv $workHome/etc/Globals.bak $file
        sed -i "2 i\ " $file
        sed -i "3 i\ \tOwnGoalColour = blue" $file
        sed -i "4 i\ \t#OwnGoalColour = yellow" $file
      fi
      if [ "$checkBlue" -o "$checkYellow" ]; then
        sed -e "/OwnGoalColour = blue/{;d;}" $file > $workHome/etc/Globals.bak
        mv  $workHome/etc/Globals.bak $file
        sed -e "/OwnGoalColour = yellow/{N;d;}" $file > $workHome/etc/Globals.bak
        mv  $workHome/etc/Globals.bak $file
        sed -i "2 i\ " $file
        sed -i "3 i\ \tOwnGoalColour = blue" $file
        sed -i "4 i\ \t#OwnGoalColour = yellow" $file
     fi
  
    else
      cp $workHome/etc/Globals.bak1 $workHome/etc/Globals.conf
      echo ""
      echo ""
      echo "     -------------------"
      echo "     |   Wrong input   |"
      echo "     |  RESET and EXIT |"
      echo "     -------------------"
      echo ""
      echo ""
      echo ""
      exit 0
    fi
  
    if [ "$inputTeam" = c ]; then
      if [ -z "$checkCyan" -o -z "$checkMagenta" ]; then
        sed -e "/OwnTeamColour = cyan/{;d;}" $file > $workHome/etc/Globals.bak
        mv $workHome/etc/Globals.bak $file
        sed -e "/OwnTeamColour = magenta/{N;d;}" $file > $workHome/etc/Globals.bak
        mv $workHome/etc/Globals.bak $file
        sed -i "5 i\ " $file
        sed -i "6 i\ \tOwnTeamColour = cyan" $file
        sed -i "7 i\ \t#OwnTeamColour = magenta" $file
      fi
      if [ "$checkCyan" -o "$checkMagenta" ]; then
        sed -e "/OwnTeamColour = cyan/{;d;}" $file > $workHome/etc/Globals.bak
        mv  $workHome/etc/Globals.bak $file
        sed -e "/OwnTeamColour = magenta/{N;d;}" $file > $workHome/etc/Globals.bak
        mv  $workHome/etc/Globals.bak $file
        sed -i "5 i\ " $file
        sed -i "6 i\ \tOwnTeamColour = cyan" $file
        sed -i "7 i\ \t#OwnTeamColour = magenta" $file
      fi
    elif [ "$inputTeam" = m ]; then
      if [ -z "$checkMagenta" -o -z "$checkCyan" ]; then
        sed -e "/OwnTeamColour = magenta/{;d;}" $file > $workHome/etc/Globals.bak
        mv $workHome/etc/Globals.bak $file
        sed -e "/OwnTeamColour = cyan/{N;d;}" $file > $workHome/etc/Globals.bak
        mv $workHome/etc/Globals.bak $file
        sed -i "5 i\ " $file
        sed -i "6 i\ \tOwnTeamColour = magenta" $file
        sed -i "7 i\ \t#OwnTeamColour = cyan" $file
      fi
      if [ "$checkMagenta" -o "$checkCyan" ]; then
        sed -e "/OwnTeamColour = magenta/{;d;}" $file > $workHome/etc/Globals.bak
        mv  $workHome/etc/Globals.bak $file
        sed -e "/OwnTeamColour = cyan/{N;d;}" $file > $workHome/etc/Globals.bak
        mv  $workHome/etc/Globals.bak $file
        sed -i "5 i\ " $file
        sed -i "6 i\ \tOwnTeamColour = magenta" $file
        sed -i "7 i\ \t#OwnTeamColour = cyan" $file
      fi
    else
      cp $workHome/etc/Globals.bak1 $workHome/etc/Globals.conf
      echo ""
      echo ""
      echo "     -------------------"
      echo "     |   Wrong input   |"
      echo "     |  RESET and EXIT |"
      echo "     -------------------"
      echo ""
      echo ""
      echo ""
      exit 0
    fi
  
    if [ -e "$file" ]; then
      head -n 8 $file
      echo ""
      echo ""
      echo "If everything ok, then press yes (y) else press no (n)" 
      echo -n "and everything will be reset: " 
      read checkInput
      if [ "$checkInput" = y ]; then
        echo ""
      elif [ "$checkInput" = n ]; then
        cp $workHome/etc/Globals.bak1 $workHome/etc/Globals.conf
        echo ""
        echo ""
        echo "     -------------------"
        echo "     |  RESET and EXIT |"
        echo "     -------------------"
        echo ""
        echo ""
        exit 0
      else
        cp $workHome/etc/Globals.bak1 $workHome/etc/Globals.conf
        echo ""
        echo ""
        echo "     -------------------"
        echo "     |   Wrong input   |"
        echo "     |  RESET and EXIT |"
        echo "     -------------------"
        echo ""
        echo ""
        echo ""
        exit 0
      fi
      rm $workHome/etc/Globals.bak1
    fi

  else
    echo ""
    echo "################################################################################"
    echo ""
    echo "   $workHome/etc/Globals.bak1"
    echo ""
    echo "   Can not be created. Please check your working directory or diskspace."
    echo ""
    echo "################################################################################"
    echo ""
  fi
  
  echo ""
  echo "-------- LOCAL COMMADS: --------"
  echo "# git pull"
  git pull
  if [[ $? != 0 ]]; then
    echo "# git pull says ERROR! $?"
  else 
    echo "# git pull says OK! $?"
    echo "# git commit $file -m "Script: Change goal color, From: $hostName""
    git commit $file -m "Script: Change goal color, From: $hostName"
    if [[ $? != 0 ]]; then
      echo "# git commit says ERROR! $?"
    else
      echo "# git commit says OK! $?"
      echo "# git push"
      git push
      if [[ $? != 0 ]]; then
        echo "# git push says ERROR! $?"
      else
        echo "# git push says OK! $?"
        echo "-------REMOTE COMMANDS: -------"
        for i in "${array[@]}"
        do
          if [ "$i" != "$hostName" ]; then
            echo "Remote commands for $i:"
            ssh cn@$i 'bash -s' < $updateGlobals
   #      ssh cn@$i "git pull $file" &
   #       ssh cn@$i 'cd $ES_ROOT; echo "";git fetch; if [ "echo $?" ]; then git checkout etc/Globals.conf; else echo "Git Fetch Fehler"; fi'
   #       ssh cn@$i 'cd $ES_ROOT; git pull'
   #        ssh cn@$i 'ls; sleep 2'
          #if [ "echo $?" ]; then
          #  echo "Connetion $i OK"
          #else
          #  echo "$i is turned off, or connection failed."
         #fi
          fi
        done
      fi 
    fi
  fi
 
else
  echo "File not exits."
  echo "This Robot will never play football ;-)"
fi
exit 0

