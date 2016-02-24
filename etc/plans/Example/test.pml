<?xml version="1.0" encoding="ASCII"?>
<alica:Plan xmi:version="2.0" xmlns:xmi="http://www.omg.org/XMI" xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance" xmlns:alica="http:///de.uni_kassel.vs.cn" id="1449767310348" name="test" comment="" masterPlan="false" utilityFunction="" utilityThreshold="0.1" destinationPath="Plans/Example" priority="0.0" minCardinality="0" maxCardinality="2147483647">
  <states id="1449767310349" name="Stop" comment="Stop" entryPoint="1449767310350">
    <plans xsi:type="alica:BehaviourConfiguration">NewStopbeh.beh#1449767995479</plans>
    <inTransitions>#1456247857100</inTransitions>
    <outTransitions>#1450351987255</outTransitions>
  </states>
  <states id="1450351941460" name="Movecenter" comment="">
    <plans xsi:type="alica:BehaviourConfiguration">../GenericBehaviours/DriveToPoint.beh#1431527260342</plans>
    <inTransitions>#1450351987255</inTransitions>
    <outTransitions>#1450351989143</outTransitions>
  </states>
  <states id="1454080477675" name="GrabTheBall" comment="">
    <plans xsi:type="alica:BehaviourConfiguration">../Behaviours/GetBall.beh#1414840399972</plans>
    <inTransitions>#1450351989143</inTransitions>
    <outTransitions>#1456247858975</outTransitions>
  </states>
  <states id="1456247693363" name="MoveToGoal" comment="">
    <plans xsi:type="alica:BehaviourConfiguration">../GenericBehaviours/DriveToPoint.beh#1443003717671</plans>
    <inTransitions>#1456247858975</inTransitions>
    <outTransitions>#1456247857100</outTransitions>
  </states>
  <transitions id="1450351987255" name="MISSING_NAME" comment="start" msg="">
    <preCondition id="1450351989007" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>#1449767310349</inState>
    <outState>#1450351941460</outState>
  </transitions>
  <transitions id="1450351989143" name="MISSING_NAME" comment="start circle" msg="">
    <preCondition id="1450351990459" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>#1450351941460</inState>
    <outState>#1454080477675</outState>
  </transitions>
  <transitions id="1456247857100" name="MISSING_NAME" comment="" msg="">
    <preCondition id="1456247858719" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>#1456247693363</inState>
    <outState>#1449767310349</outState>
  </transitions>
  <transitions id="1456247858975" name="MISSING_NAME" comment="" msg="">
    <preCondition id="1456247861415" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>#1454080477675</inState>
    <outState>#1456247693363</outState>
  </transitions>
  <entryPoints id="1449767310350" name="MISSING_NAME" comment="" successRequired="false" minCardinality="0" maxCardinality="2147483647">
    <task>../../Misc/taskrepository.tsk#1225112227903</task>
    <state>#1449767310349</state>
  </entryPoints>
</alica:Plan>
