<?xml version="1.0" encoding="ASCII"?>
<alica:Plan xmi:version="2.0" xmlns:xmi="http://www.omg.org/XMI" xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance" xmlns:alica="http:///de.uni_kassel.vs.cn" id="1467383326416" name="OwnStdSingleRobot" comment="" destinationPath="Plans/Standards/Own" priority="0.0" minCardinality="1" maxCardinality="1" masterPlan="false" utilityFunction="" utilityThreshold="0.1">
  <states id="1467384083266" name="Position" comment="" entryPoint="1467384083267">
    <plans xsi:type="alica:Behaviour">Standards/Own/StdAlignSingleRobot.beh#1467385758084</plans>
    <outTransitions>Standards/Own/OwnStdSingleRobot.pml#1467384202351</outTransitions>
  </states>
  <states id="1467384130052" name="GetBall" comment="">
    <plans xsi:type="alica:Behaviour">GenericStandards/StandardAlignAndGrab.beh#1455888574532</plans>
    <inTransitions>Standards/Own/OwnStdSingleRobot.pml#1467384202351</inTransitions>
    <outTransitions>Standards/Own/OwnStdSingleRobot.pml#1467384215357</outTransitions>
  </states>
  <states id="1467384133137" name="Kick" comment="">
    <plans xsi:type="alica:Behaviour">Standards/Own/SingleRobotKickIntoOppHalf.beh#1467436234548</plans>
    <inTransitions>Standards/Own/OwnStdSingleRobot.pml#1467384215357</inTransitions>
    <outTransitions>Standards/Own/OwnStdSingleRobot.pml#1467384963126</outTransitions>
  </states>
  <states xsi:type="alica:SuccessState" id="1467384222671" name="NewSuccessState" comment="">
    <inTransitions>Standards/Own/OwnStdSingleRobot.pml#1467384963126</inTransitions>
  </states>
  <transitions id="1467384202351" name="MISSING_NAME" comment="situation start" msg="">
    <preCondition id="1467384214949" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>Standards/Own/OwnStdSingleRobot.pml#1467384083266</inState>
    <outState>Standards/Own/OwnStdSingleRobot.pml#1467384130052</outState>
  </transitions>
  <transitions id="1467384215357" name="MISSING_NAME" comment="haveball" msg="">
    <preCondition id="1467384216416" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>Standards/Own/OwnStdSingleRobot.pml#1467384130052</inState>
    <outState>Standards/Own/OwnStdSingleRobot.pml#1467384133137</outState>
  </transitions>
  <transitions id="1467384963126" name="MISSING_NAME" comment="kicked" msg="">
    <preCondition id="1467384969800" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>Standards/Own/OwnStdSingleRobot.pml#1467384133137</inState>
    <outState>Standards/Own/OwnStdSingleRobot.pml#1467384222671</outState>
  </transitions>
  <entryPoints id="1467384083267" name="MISSING_NAME" comment="" successRequired="true" minCardinality="1" maxCardinality="1">
    <task>taskrepository.tsk#1225112227903</task>
    <state>Standards/Own/OwnStdSingleRobot.pml#1467384083266</state>
  </entryPoints>
</alica:Plan>
