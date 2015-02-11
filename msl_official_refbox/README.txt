The package contains the MSL refbox.

The website of this project is: http://sourceforge.net/projects/msl-refbox
On the website the latest version is available. 

The root-address of the SVN repository of this project is: 
https://msl-refbox.svn.sourceforge.net/svnroot/msl-refbox

The latest development status can be found on:
https://msl-refbox.svn.sourceforge.net/svnroot/msl-refbox/trunk


The MSL refbox consists of a few tools.

The following tools are currently part of the MSL refbox.
- rb-gui-main-2013-2.02.jar
  This is the main refbox application.
- rb-gui-status-2013-2.02.jar
  This program displays the information received from the refbox. Only working
  if the refbox is using a multicast of the protocol 2008 (xml)
- rb-gui-teamsetup-2013-2.02.jar
  This program can be used to send the teaminformation before the start of the game to the refbox.
- rb-gui-testclient2009-2013-2.02.jar
  This program can be used to display the retrieved commands via protocol2009 (character based as used in Graz)
- rb-gui-testclient2010-2013-2.02.jar
  This program can be used to display the retrieved commands via protocol2010 (xml based)


For running the MSL refbox at least Java 6 must be available.
This can be verified with command "java -version" on the commandline.

Start a tool with: "java -jar <tool>.jar" on the commandline.
par example: java -jar rb-gui-testclient2010-2013-2.02.jar

