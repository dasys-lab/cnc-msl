<?xml version="1.0" encoding="ISO-8859-1"?>
<xsl:stylesheet version="1.0" xmlns:xsl="http://www.w3.org/1999/XSL/Transform">
<xsl:output method="html"/>


<!-- Refbox Main begin-->
<xsl:template match="/Game">
  <HTML>
	  <HEAD>
		  <title>Game Sheet</title>

<STYLE TYPE="text/css">
.title {
			font-weight:	bold
}
</STYLE>			

		</HEAD>
    <BODY> 
	    <xsl:call-template name="title"/>
    	<xsl:apply-templates select="./Teams"/>
	    <xsl:call-template name="score"/>
			<xsl:apply-templates select="./Referee"/>
	    <xsl:apply-templates select="./Stats" mode="Cards"/>
	    <xsl:apply-templates select="./Stats" mode="Goals"/>
	    <xsl:apply-templates select="./Stats" mode="Stats"/>
    </BODY>
  </HTML>
</xsl:template>
<!-- Dialoog Main end-->

<!-- Title begin -->
<xsl:template name="title">
	<p>
  <h1>MSL RoboCup Game</h1>  
	<TABLE cellSpacing="0" cellPadding="3" width="100%" align="center" border="0">
  	<TBODY>
  		<TR>
    		<TD width="25%"><b>Field</b></TD>
    		<TD width="25%"><xsl:apply-templates select="/Game/Location"/></TD>
    		<TD width="25%"></TD>
    		<TD width="25%"></TD>
      </TR>
      <TR>
    		<TD width="25%"><b>Date</b></TD>
    		<TD width="25%"><xsl:apply-templates select="/Game/Date"/></TD>
    		<TD width="25%"><b>Time</b></TD>
    		<TD width="25%"><xsl:apply-templates select="/Game/Time"/></TD>
			</TR>
		</TBODY>
	</TABLE>
  </p>
</xsl:template>

<!-- Teams begin -->
<xsl:template match="Teams">
<p>
<b>Teams</b>
  <TABLE cellSpacing="0" cellPadding="3" width="100%" align="center" border="1">
  	<TBODY>
  		<TR>
    		<TD colSpan="2">Team: <b><xsl:apply-templates select="./SetupA/Teamname"/></b></TD>
    		<TD colSpan="2">Team: <b><xsl:apply-templates select="./SetupB/Teamname"/></b></TD>
			</TR>
  		<TR>
    		<TD width="20%">Team Leader</TD>
    		<TD width="30%"><xsl:apply-templates select="./SetupA/Teamleader"/></TD>
    		<TD width="20%">Team Leader</TD>
    		<TD width="30%"><xsl:apply-templates select="./SetupB/Teamleader"/></TD>
			</TR>
  		<TR>
    		<TD width="20%">Team Color</TD>
    		<TD width="30%"><xsl:apply-templates select="./SetupA/Teamcolor"/></TD>
    		<TD width="20%">Team Color</TD>
    		<TD width="30%"><xsl:apply-templates select="./SetupB/Teamcolor"/></TD>
			</TR>
		</TBODY>
	</TABLE>
</p>
</xsl:template>

<xsl:template match="/Game/SetupA/Teamname|/Game/SetupB/Teamname">
  <xsl:value-of select="."/>
</xsl:template>

<xsl:template match="/Game/SetupA/Teamleader|/Game/SetupB/Teamleader">
  <xsl:value-of select="."/>
</xsl:template>

<xsl:template match="/Game/SetupA/Teamcolor|/Game/SetupB/Teamcolor">
  <xsl:value-of select="."/>
</xsl:template>

<xsl:template name="score">
	<p>
  <b>Final Score</b> 
	<TABLE cellSpacing="0" cellPadding="3" width="100%" align="center" border="1">
  	<TBODY>
  		<TR>
    		<TD width="20%"></TD>
    		<TD width="30%"><b><xsl:value-of select="count(/Game/Stats/TeamA/Goals/*)"/></b></TD>
    		<TD width="20%"></TD>
    		<TD width="30%"><b><xsl:value-of select="count(/Game/Stats/teamB/Goals/*)"/></b></TD>
			</TR>
		</TBODY>
	</TABLE>
  </p>
</xsl:template>

<xsl:template match="Referee">
	<p>
  <b>Referees</b> 
	<TABLE cellSpacing="0" cellPadding="3" width="100%" align="center" border="1">
  	<TBODY>
  		<TR>
    		<TD width="25%">Main-referee</TD>
    		<TD width="25%"><xsl:apply-templates select="./Main-referee"/></TD>
    		<TD width="25%">Time Keeper </TD>
    		<TD width="25%"><xsl:apply-templates select="./Time-Keeper"/></TD>
			</TR>
  		<TR>
    		<TD width="25%">Assistant Referee 1</TD>
    		<TD width="25%"><xsl:apply-templates select="./Assistant-Referee-1"/></TD>
    		<TD width="25%">Assistant Referee 2</TD>
    		<TD width="25%"><xsl:apply-templates select="./Assistant-Referee-2"/></TD>
			</TR>
  		<TR>
    		<TD>1st Additional Assistant Referee</TD>
    		<TD><xsl:apply-templates select="./Additional-Assistant-Referee-1"/></TD>
    		<TD>2nd Additional Assistant Referee</TD>
    		<TD><xsl:apply-templates select="./Additional-Assistant-Referee-2"/></TD>
			</TR>
		</TBODY>
	</TABLE>
  </p>
</xsl:template>

<xsl:template match="/Game/Referee/Main-Referee|/Game/Referee/Time-Keeper|/Game/Referee/Assistant-Referee-1|/Game/Referee/Assistant-Referee-2|/game/referee/Additional-Assistant-Referee-1|/game/referee/Additional-Assistant-Referee-2">
  <xsl:value-of select="."/>
</xsl:template>


<xsl:template match="/Game/Stats" mode="Cards">
<p><b>Cards</b> 
<TABLE cellSpacing="0" cellPadding="3" width="100%" align="center" border="1">
  <TBODY>
		<TR>
			<TD width="50%" valign="top">
				<xsl:apply-templates select="./TeamA/Cards"/>
			</TD>
			<TD width="50%" valign="top">
				<xsl:apply-templates select="./TeamB/Cards"/>
			</TD>
		</TR>
	</TBODY>
</TABLE>				
</p>
</xsl:template>

<xsl:template match="/Game/Stats/*/Cards">
	<TABLE cellSpacing="0" cellPadding="3" width="100%" align="center" border="1">
  	<TBODY>
  		<TR class="title">
    		<TD width="25%">Player #</TD>
    		<TD width="25%">Stage</TD>
    		<TD width="25%">Time</TD>
    		<TD width="25%">Color</TD>
			</TR>
			<xsl:apply-templates select="./Card"/>
		</TBODY>
	</TABLE>
</xsl:template>

<xsl:template match="/Game/Stats/*/Cards/Card">
			<TR>
				<TD width="25%"><xsl:value-of select="./Player"/></TD>
				<TD width="25%"><xsl:value-of select="./Stage"/></TD>
				<TD width="25%"><xsl:value-of select="./Time"/></TD>
				<TD width="25%"><xsl:value-of select="./Color"/></TD>
			</TR>	
</xsl:template>

<xsl:template match="/Game/Stats" mode="Goals">
<p>
<b>Goals</b>
<TABLE cellSpacing="0" cellPadding="3" width="100%" align="center" border="1">
  <TBODY>
		<TR>
			<TD width="50%" valign="top">
				<xsl:apply-templates select="./TeamA/Goals"/>
			</TD>
			<TD width="50%" valign="top">
				<xsl:apply-templates select="./TeamB/Goals"/>
			</TD>
		</TR>
	</TBODY>
</TABLE>				
</p>
</xsl:template>

<xsl:template match="/Game/Stats/*/Goals">
  <TABLE cellSpacing="0" cellPadding="3" width="100%" align="center" border="1">
  	<TBODY>
  		<TR class="title">
    		<TD width="25%">Player #</TD>
    		<TD width="25%">Stage</TD>
    		<TD width="25%">Time</TD>
			</TR>
			<xsl:apply-templates select="./Goal"/>
		</TBODY>
	</TABLE>
</xsl:template>

<xsl:template match="/Game/Stats/*/Goals/Goal">
			<TR>
				<TD width="25%"><xsl:value-of select="./Player"/>
				<xsl:if test="./Own = 'true'"> (own goal)</xsl:if></TD>
				<TD width="25%"><xsl:value-of select="./Stage"/></TD>
				<TD width="25%"><xsl:value-of select="./Time"/></TD>
			</TR>	
</xsl:template>

<xsl:template match="/Game/Stats" mode="Stats">
<p>
<b>Stats</b>
<TABLE cellSpacing="0" cellPadding="3" width="100%" align="center" border="1">
  <TBODY>
		 <TR>
			 <TD width="25%">first half</TD>
       <TD width="25%"><xsl:value-of select="./General/Times/FirstHalf"/></TD>
			 <TD width="25%">play time</TD>
       <TD width="25%"><xsl:value-of select="./General/Times/FirstHalfPlayed"/></TD>
     </TR>
		 <TR>
			 <TD width="25%">second half</TD>
       <TD width="25%"><xsl:value-of select="./General/Times/SecondHalf"/></TD>
			 <TD width="25%">play time</TD>
       <TD width="25%"><xsl:value-of select="./General/Times/SecondHalfPlayed"/></TD>
     </TR>
		<TR> 
       <TD width="25%">pregame</TD>
       <TD width="25%"><xsl:value-of select="./General/Times/Pregame"/></TD>
       <TD width="25%">half time</TD>
       <TD width="25%"><xsl:value-of select="./General/Times/HalfTime"/></TD>
     </TR>
		 <TR>
			 <TD width="25%">neutral starts</TD>
       <TD width="25%"><xsl:value-of select="./General/NeutralStarts"/></TD>
			 <TD width="25%">dropped balls</TD>
       <TD width="25%"><xsl:value-of select="./General/DroppedBalls"/></TD>
     </TR>
  </TBODY>
</TABLE>
</p>

<p>
<b>Team Stats</b>
<TABLE cellSpacing="0" cellPadding="3" width="100%" align="center" border="0">
  <TBODY>
		<TR>
			<TD width="50%" valign="top">
        <TABLE cellSpacing="0" cellPadding="3" width="100%" align="center" border="1">
          <TBODY>
		        <TR>
			        <TD>kickoffs</TD>
              <TD><xsl:value-of select="./TeamA/Kickoffs"/></TD>
            </TR>
		        <TR>
			        <TD>freekicks</TD>
              <TD><xsl:value-of select="./TeamA/Freekicks"/></TD>
            </TR>
		        <TR>
			        <TD>goalkicks</TD>
              <TD><xsl:value-of select="./TeamA/Goalkicks"/></TD>
            </TR>
		        <TR>
			        <TD>corners</TD>
              <TD><xsl:value-of select="./TeamA/Corners"/></TD>
            </TR>
		        <TR>
			        <TD>throwins</TD>
              <TD><xsl:value-of select="./TeamA/Throwins"/></TD>
            </TR>
		        <TR>
			        <TD>penalties</TD>
              <TD><xsl:value-of select="./TeamA/Penalties"/></TD>
            </TR>
          </TBODY>
        </TABLE>
			</TD>
			<TD width="50%" valign="top">
        <TABLE cellSpacing="0" cellPadding="3" width="100%" align="center" border="1">
          <TBODY>
		        <TR>
			        <TD>kickoffs</TD>
              <TD><xsl:value-of select="./TeamB/Kickoffs"/></TD>
            </TR>
		        <TR>
			        <TD>freekicks</TD>
              <TD><xsl:value-of select="./TeamB/Freekicks"/></TD>
            </TR>
		        <TR>
			        <TD>goalkicks</TD>
              <TD><xsl:value-of select="./TeamB/Goalkicks"/></TD>
            </TR>
		        <TR>
			        <TD>corners</TD>
              <TD><xsl:value-of select="./TeamB/Corners"/></TD>
            </TR>
		        <TR>
			        <TD>throwins</TD>
              <TD><xsl:value-of select="./TeamB/Throwins"/></TD>
            </TR>
		        <TR>
			        <TD>penalties</TD>
              <TD><xsl:value-of select="./TeamB/Penalties"/></TD>
            </TR>
          </TBODY>
        </TABLE>
			</TD>
		</TR>
	</TBODY>
</TABLE>				
</p>
</xsl:template>


</xsl:stylesheet>
