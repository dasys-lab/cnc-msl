<?xml version="1.0" encoding="ASCII"?>
<alica:Plan xmi:version="2.0" xmlns:xmi="http://www.omg.org/XMI" xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance" xmlns:alica="http:///de.uni_kassel.vs.cn" id="1434046634656" name="StandardAttack" comment="lostBall" masterPlan="false" utilityFunction="" utilityThreshold="0.1" destinationPath="Plans/Attack" priority="0.0" minCardinality="1" maxCardinality="1">
  <states id="1434046634657" name="GetBall" comment="" entryPoint="1434046634658">
    <plans xsi:type="alica:BehaviourConfiguration">../Dribble/DribbleControl.beh#1450175539163</plans>
    <plans xsi:type="alica:BehaviourConfiguration">../Behaviours/Intercept.beh#1458757193843</plans>
    <inTransitions>#1434048722503</inTransitions>
    <inTransitions>#1450175866364</inTransitions>
    <inTransitions>#1450175921044</inTransitions>
    <inTransitions>#1457687426425</inTransitions>
    <outTransitions>#1434048720937</outTransitions>
    <outTransitions>#1434716047438</outTransitions>
    <outTransitions>#1450175864724</outTransitions>
  </states>
  <states id="1434048406725" name="Duel" comment="">
    <plans xsi:type="alica:BehaviourConfiguration">../Behaviours/Duel.beh#1450178707835</plans>
    <inTransitions>#1434048729645</inTransitions>
    <outTransitions>#1434048734889</outTransitions>
    <outTransitions>#1457687426425</outTransitions>
  </states>
  <states id="1434048705508" name="HaveBall" comment="">
    <plans xsi:type="alica:PlanType">PassPlayType.pty#1508950344228</plans>
    <inTransitions>#1434048720937</inTransitions>
    <inTransitions>#1434048734889</inTransitions>
    <inTransitions>#1434716048579</inTransitions>
    <outTransitions>#1434048722503</outTransitions>
    <outTransitions>#1434048729645</outTransitions>
    <outTransitions>#1434716049424</outTransitions>
    <outTransitions>#1450175926476</outTransitions>
  </states>
  <states id="1434715893346" name="LostBall" comment="">
    <plans xsi:type="alica:BehaviourConfiguration">Wander.beh#1434716230628</plans>
    <inTransitions>#1434716047438</inTransitions>
    <inTransitions>#1434716049424</inTransitions>
    <outTransitions>#1434716048579</outTransitions>
  </states>
  <states id="1450175617600" name="SideLineGetBall" comment="">
    <plans xsi:type="alica:BehaviourConfiguration">../Dribble/DribbleControl.beh#1450175539163</plans>
    <plans xsi:type="alica:BehaviourConfiguration">FetchFromSideLine.beh#1450175679178</plans>
    <inTransitions>#1450175864724</inTransitions>
    <outTransitions>#1450175866364</outTransitions>
  </states>
  <states id="1450175887984" name="AdvanceAfterPass" comment="">
    <plans xsi:type="alica:BehaviourConfiguration">AdvancdeSimplePass.beh#1450176216458</plans>
    <inTransitions>#1450175926476</inTransitions>
    <outTransitions>#1450175921044</outTransitions>
  </states>
  <transitions id="1434048720937" name="MISSING_NAME" comment="haveBall" msg="">
    <preCondition id="1434048722207" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>#1434046634657</inState>
    <outState>#1434048705508</outState>
  </transitions>
  <transitions id="1434048722503" name="MISSING_NAME" comment="lostBall" msg="">
    <preCondition id="1434048723635" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>#1434048705508</inState>
    <outState>#1434046634657</outState>
  </transitions>
  <transitions id="1434048729645" name="MISSING_NAME" comment="haveBall &amp;&amp; enemy close" msg="">
    <preCondition id="1434048731181" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>#1434048705508</inState>
    <outState>#1434048406725</outState>
  </transitions>
  <transitions id="1434048734889" name="MISSING_NAME" comment="haveball &amp;&amp; enemy not close" msg="">
    <preCondition id="1434048737070" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>#1434048406725</inState>
    <outState>#1434048705508</outState>
  </transitions>
  <transitions id="1434716047438" name="MISSING_NAME" comment="doesnt see ball || the opponent in ballpossesion" msg="">
    <preCondition id="1434716048353" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>#1434046634657</inState>
    <outState>#1434715893346</outState>
  </transitions>
  <transitions id="1434716048579" name="MISSING_NAME" comment="found ball" msg="">
    <preCondition id="1434716049299" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>#1434715893346</inState>
    <outState>#1434048705508</outState>
  </transitions>
  <transitions id="1434716049424" name="MISSING_NAME" comment="(doesnt see ball || the opponent in ball possesion) &amp;&amp; wm->whiteBoard.getPassMsg() != nullptr" msg="">
    <preCondition id="1434716050319" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>#1434048705508</inState>
    <outState>#1434715893346</outState>
  </transitions>
  <transitions id="1450175864724" name="MISSING_NAME" comment="Ball at sideline and not behind ball and no opponent near Ball" msg="">
    <preCondition id="1450175866027" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>#1434046634657</inState>
    <outState>#1450175617600</outState>
  </transitions>
  <transitions id="1450175866364" name="MISSING_NAME" comment="SideLineGetBall2GetBall: Ball not at sideline or Fail or opponent near Ball || SideLineGetBall2ApproachBall: Success || dont see the ball" msg="">
    <preCondition id="1450175867019" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>#1450175617600</inState>
    <outState>#1434046634657</outState>
  </transitions>
  <transitions id="1450175921044" name="MISSING_NAME" comment="KickSomeHowFailed -> We need to reaproach the ball" msg="">
    <preCondition id="1450175926118" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>#1450175887984</inState>
    <outState>#1434046634657</outState>
  </transitions>
  <transitions id="1450175926476" name="MISSING_NAME" comment="not HaveBallDribble(true) &amp;&amp; PassMsg!=null" msg="">
    <preCondition id="1450175931490" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>#1434048705508</inState>
    <outState>#1450175887984</outState>
  </transitions>
  <transitions id="1457687426425" name="MISSING_NAME" comment="ballpos == nullptr " msg="">
    <preCondition id="1457687428316" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>#1434048406725</inState>
    <outState>#1434046634657</outState>
  </transitions>
  <entryPoints id="1434046634658" name="Attack" comment="" successRequired="false" minCardinality="1" maxCardinality="1">
    <task>../../Misc/taskrepository.tsk#1222613952469</task>
    <state>#1434046634657</state>
  </entryPoints>
</alica:Plan>
