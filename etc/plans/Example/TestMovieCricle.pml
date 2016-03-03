<?xml version="1.0" encoding="ASCII"?>
<alica:Plan xmi:version="2.0" xmlns:xmi="http://www.omg.org/XMI" xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance" xmlns:alica="http:///de.uni_kassel.vs.cn" id="1450104551642" name="TestMovieCricle" comment="" masterPlan="false" utilityFunction="" utilityThreshold="0.1" destinationPath="Plans/Example" priority="0.0" minCardinality="0" maxCardinality="2147483647">
  <states id="1450104551643" name="Stop" comment="" entryPoint="1450104551644">
    <plans xsi:type="alica:BehaviourConfiguration">NewStopbeh.beh#1449767995479</plans>
    <inTransitions>#1456754697400</inTransitions>
    <outTransitions>#1450104564490</outTransitions>
  </states>
  <states id="1450104559962" name="ThaoRotateCircle" comment="RotateCircle360 degree">
    <plans xsi:type="alica:BehaviourConfiguration">ThaoRotateCircle.beh#1450104636697</plans>
    <inTransitions>#1450104564490</inTransitions>
    <outTransitions>#1456754697400</outTransitions>
  </states>
  <transitions id="1450104564490" name="MISSING_NAME" comment="" msg="">
    <preCondition id="1450104566808" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>#1450104551643</inState>
    <outState>#1450104559962</outState>
  </transitions>
  <transitions id="1456754697400" name="MISSING_NAME" comment="" msg="">
    <preCondition id="1456754698846" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>#1450104559962</inState>
    <outState>#1450104551643</outState>
  </transitions>
  <entryPoints id="1450104551644" name="MISSING_NAME" comment="" successRequired="false" minCardinality="0" maxCardinality="2147483647">
    <task>../../Misc/taskrepository.tsk#1225112227903</task>
    <state>#1450104551643</state>
  </entryPoints>
</alica:Plan>
