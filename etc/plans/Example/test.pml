<?xml version="1.0" encoding="ASCII"?>
<alica:Plan xmi:version="2.0" xmlns:xmi="http://www.omg.org/XMI" xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance" xmlns:alica="http:///de.uni_kassel.vs.cn" id="1449767310348" name="test" comment="" masterPlan="false" utilityFunction="" utilityThreshold="0.1" destinationPath="Plans/Example" priority="0.0" minCardinality="0" maxCardinality="2147483647">
  <states id="1449767310349" name="Stop" comment="Stop" entryPoint="1449767310350">
    <plans xsi:type="alica:BehaviourConfiguration">NewStopbeh.beh#1449767995479</plans>
    <inTransitions>#1456847658915</inTransitions>
    <outTransitions>#1450351987255</outTransitions>
  </states>
  <states id="1454080477675" name="GrabTheBall" comment="">
    <plans xsi:type="alica:BehaviourConfiguration">../Behaviours/StdExecutorGrabBall.beh#1441209089978</plans>
    <inTransitions>#1450351987255</inTransitions>
    <outTransitions>#1456847658915</outTransitions>
  </states>
  <transitions id="1450351987255" name="MISSING_NAME" comment="start" msg="">
    <preCondition id="1450351989007" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>#1449767310349</inState>
    <outState>#1454080477675</outState>
  </transitions>
  <transitions id="1456847658915" name="MISSING_NAME" comment="" msg="">
    <preCondition id="1456847660301" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>#1454080477675</inState>
    <outState>#1449767310349</outState>
  </transitions>
  <entryPoints id="1449767310350" name="MISSING_NAME" comment="" successRequired="false" minCardinality="0" maxCardinality="2147483647">
    <task>../../Misc/taskrepository.tsk#1225112227903</task>
    <state>#1449767310349</state>
  </entryPoints>
</alica:Plan>
