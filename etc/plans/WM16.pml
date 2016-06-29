<?xml version="1.0" encoding="ASCII"?>
<alica:Plan xmi:version="2.0" xmlns:xmi="http://www.omg.org/XMI" xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance" xmlns:alica="http:///de.uni_kassel.vs.cn" id="1413992564408" name="WM16" comment="" masterPlan="true" utilityFunction="" utilityThreshold="0.1" destinationPath="Plans" priority="0.0" minCardinality="0" maxCardinality="2147483647">
  <states id="1413992564409" name="Stop" comment="" entryPoint="1413992564410">
    <plans xsi:type="alica:BehaviourConfiguration">GenericBehaviours/Stop.beh#1413992626194</plans>
    <inTransitions>#1426692315498</inTransitions>
    <inTransitions>#1426692322651</inTransitions>
    <inTransitions>#1426692324690</inTransitions>
    <inTransitions>#1426692326813</inTransitions>
    <inTransitions>#1426692328789</inTransitions>
    <inTransitions>#1426692331032</inTransitions>
    <outTransitions>#1426691974126</outTransitions>
    <outTransitions>#1426691976989</outTransitions>
    <outTransitions>#1426691979458</outTransitions>
    <outTransitions>#1426692077573</outTransitions>
    <outTransitions>#1426692164404</outTransitions>
    <outTransitions>#1426692244795</outTransitions>
  </states>
  <states id="1426691835473" name="Gameplay" comment="">
    <plans xsi:type="alica:Plan">GameStrategy/Gameplay/GamePlay.pml#1457173546734</plans>
    <inTransitions>#1426691974126</inTransitions>
    <inTransitions>#1426692306144</inTransitions>
    <inTransitions>#1426692308143</inTransitions>
    <inTransitions>#1426692653093</inTransitions>
    <outTransitions>#1426692324690</outTransitions>
  </states>
  <states id="1426691842031" name="OpponentStandard" comment="">
    <plans xsi:type="alica:Plan">GenericStandards/GenericOppStandards.pml#1432132075122</plans>
    <inTransitions>#1426691979458</inTransitions>
    <outTransitions>#1426692308143</outTransitions>
    <outTransitions>#1426692328789</outTransitions>
  </states>
  <states id="1426691966433" name="OwnStandard" comment="">
    <plans xsi:type="alica:Plan">GenericStandards/GenericOwnStandards.pml#1430924951132</plans>
    <inTransitions>#1426691976989</inTransitions>
    <outTransitions>#1426692306144</outTransitions>
    <outTransitions>#1426692326813</outTransitions>
  </states>
  <states id="1426692070119" name="Joystick" comment="">
    <plans xsi:type="alica:BehaviourConfiguration">Behaviours/Joystick.beh#1421854995808</plans>
    <plans xsi:type="alica:BehaviourConfiguration">Dribble/DribbleControl.beh#1449742099555</plans>
    <inTransitions>#1426692077573</inTransitions>
    <outTransitions>#1426692322651</outTransitions>
  </states>
  <states id="1426692157956" name="Parking" comment="">
    <plans xsi:type="alica:Plan">GameStrategy/Other/Parking.pml#1426695119330</plans>
    <inTransitions>#1426692164404</inTransitions>
    <outTransitions>#1426692331032</outTransitions>
  </states>
  <states id="1426692210342" name="DroppedBall" comment="">
    <plans xsi:type="alica:Plan">GameStrategy/Other/DroppedBall.pml#1426694906399</plans>
    <inTransitions>#1426692244795</inTransitions>
    <outTransitions>#1426692315498</outTransitions>
    <outTransitions>#1426692653093</outTransitions>
  </states>
  <transitions id="1426691974126" name="MISSING_NAME" comment="situation == start" msg="">
    <preCondition id="1426691976695" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>#1413992564409</inState>
    <outState>#1426691835473</outState>
  </transitions>
  <transitions id="1426691976989" name="MISSING_NAME" comment="situation == Own(KickOff || FreeKick || GoalKick || Throwin || Corner || Penality)" msg="">
    <preCondition id="1426691979272" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>#1413992564409</inState>
    <outState>#1426691966433</outState>
  </transitions>
  <transitions id="1426691979458" name="MISSING_NAME" comment="situation == Opp(KickOff || FreeKick || GoalKick || Throwin || Corner || Penality)" msg="">
    <preCondition id="1426691980585" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>#1413992564409</inState>
    <outState>#1426691842031</outState>
  </transitions>
  <transitions id="1426692077573" name="MISSING_NAME" comment="situation == joystick" msg="">
    <preCondition id="1426692078983" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>#1413992564409</inState>
    <outState>#1426692070119</outState>
  </transitions>
  <transitions id="1426692164404" name="MISSING_NAME" comment="situation == parking" msg="">
    <preCondition id="1426692165988" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>#1413992564409</inState>
    <outState>#1426692157956</outState>
  </transitions>
  <transitions id="1426692244795" name="MISSING_NAME" comment="situation == dropBall" msg="">
    <preCondition id="1426692246279" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>#1413992564409</inState>
    <outState>#1426692210342</outState>
  </transitions>
  <transitions id="1426692306144" name="From Own Standard to GamePlay" comment="anyChildSuccess ||  moreThen7SecondsInStart" msg="">
    <preCondition id="1426692307991" name="Condition an &quot;From OwnStandard to GamePlay&quot;" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>#1426691966433</inState>
    <outState>#1426691835473</outState>
  </transitions>
  <transitions id="1426692308143" name="MISSING_NAME" comment="anyChildSuccess || moreThen7SecondsInStart" msg="">
    <preCondition id="1426692309848" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>#1426691842031</inState>
    <outState>#1426691835473</outState>
  </transitions>
  <transitions id="1426692315498" name="MISSING_NAME" comment="situation  != DropBall &amp;&amp; situation !=start" msg="">
    <preCondition id="1426692322491" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>#1426692210342</inState>
    <outState>#1413992564409</outState>
  </transitions>
  <transitions id="1426692322651" name="MISSING_NAME" comment="situation == !joystick" msg="">
    <preCondition id="1426692324569" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>#1426692070119</inState>
    <outState>#1413992564409</outState>
  </transitions>
  <transitions id="1426692324690" name="MISSING_NAME" comment="situation != start" msg="">
    <preCondition id="1426692326693" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>#1426691835473</inState>
    <outState>#1413992564409</outState>
  </transitions>
  <transitions id="1426692326813" name="MISSING_NAME" comment="situation != standard own situation || stop" msg="">
    <preCondition id="1426692328653" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>#1426691966433</inState>
    <outState>#1413992564409</outState>
  </transitions>
  <transitions id="1426692328789" name="MISSING_NAME" comment="situation != opp standard situation || start" msg="">
    <preCondition id="1426692330833" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>#1426691842031</inState>
    <outState>#1413992564409</outState>
  </transitions>
  <transitions id="1426692331032" name="MISSING_NAME" comment="situation != parken " msg="">
    <preCondition id="1426692332679" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>#1426692157956</inState>
    <outState>#1413992564409</outState>
  </transitions>
  <transitions id="1426692653093" name="MISSING_NAME" comment="anyChildSuccess || moreThen7SecondsInStart" msg="">
    <preCondition id="1426692655810" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>#1426692210342</inState>
    <outState>#1426691835473</outState>
  </transitions>
  <entryPoints id="1413992564410" name="" comment="" successRequired="false" minCardinality="0" maxCardinality="2147483647">
    <task>../Misc/taskrepository.tsk#1225112227903</task>
    <state>#1413992564409</state>
  </entryPoints>
</alica:Plan>
