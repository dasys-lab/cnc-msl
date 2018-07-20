<?xml version="1.0" encoding="ASCII"?>
<alica:Plan xmi:version="2.0" xmlns:xmi="http://www.omg.org/XMI" xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance" xmlns:alica="http:///de.uni_kassel.vs.cn" id="1532092691315" name="TestGoalCoordinates" comment="" masterPlan="true" utilityFunction="" utilityThreshold="0.1" destinationPath="Plans" priority="0.0" minCardinality="0" maxCardinality="2147483647">
  <states id="1532092691316" name="Stop" comment="" entryPoint="1532092691317">
    <inTransitions>#1532092759824</inTransitions>
    <outTransitions>#1532092762539</outTransitions>
  </states>
  <states id="1532092743927" name="NewState" comment="">
    <plans xsi:type="alica:BehaviourConfiguration">TestPlans/TestGoalCoordinates.beh#1532092891925</plans>
    <inTransitions>#1532092762539</inTransitions>
    <outTransitions>#1532092759824</outTransitions>
  </states>
  <transitions id="1532092759824" name="MISSING_NAME" comment="stop" msg="">
    <preCondition id="1532092762346" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>#1532092743927</inState>
    <outState>#1532092691316</outState>
  </transitions>
  <transitions id="1532092762539" name="MISSING_NAME" comment="start" msg="">
    <preCondition id="1532092768263" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>#1532092691316</inState>
    <outState>#1532092743927</outState>
  </transitions>
  <entryPoints id="1532092691317" name="MISSING_NAME" comment="" successRequired="false" minCardinality="0" maxCardinality="2147483647">
    <task>../Misc/taskrepository.tsk#1225112227903</task>
    <state>#1532092691316</state>
  </entryPoints>
</alica:Plan>
