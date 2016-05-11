<?xml version="1.0" encoding="ASCII"?>
<alica:Plan xmi:version="2.0" xmlns:xmi="http://www.omg.org/XMI" xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance" xmlns:alica="http:///de.uni_kassel.vs.cn" id="1462969665131" name="RobotMovementTestPlan" comment="" masterPlan="false" utilityFunction="" utilityThreshold="0.1" destinationPath="Plans/Attack/TestPlans" priority="0.0" minCardinality="0" maxCardinality="2147483647">
  <states id="1462969665132" name="Stop" comment="" entryPoint="1462969665133">
    <plans xsi:type="alica:BehaviourConfiguration">../../GenericBehaviours/Stop.beh#1413992626194</plans>
    <inTransitions>#1462969786619</inTransitions>
    <inTransitions>#1462969919287</inTransitions>
    <outTransitions>#1462969759156</outTransitions>
  </states>
  <states id="1462969698097" name="DribbleTest" comment="">
    <plans xsi:type="alica:BehaviourConfiguration">../../Behaviours/RobotMovementDribbleTest.beh#1462969753310</plans>
    <inTransitions>#1462969917381</inTransitions>
    <outTransitions>#1462969919287</outTransitions>
  </states>
  <states id="1462969884284" name="moveToBall" comment="">
    <plans xsi:type="alica:BehaviourConfiguration">../../Behaviours/GetBall.beh#1414840399972</plans>
    <inTransitions>#1462969759156</inTransitions>
    <outTransitions>#1462969786619</outTransitions>
    <outTransitions>#1462969917381</outTransitions>
  </states>
  <transitions id="1462969759156" name="MISSING_NAME" comment="start" msg="">
    <preCondition id="1462969760665" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>#1462969665132</inState>
    <outState>#1462969884284</outState>
  </transitions>
  <transitions id="1462969786619" name="MISSING_NAME" comment="stop" msg="">
    <preCondition id="1462969788458" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>#1462969884284</inState>
    <outState>#1462969665132</outState>
  </transitions>
  <transitions id="1462969917381" name="MISSING_NAME" comment="have ball" msg="">
    <preCondition id="1462969919094" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>#1462969884284</inState>
    <outState>#1462969698097</outState>
  </transitions>
  <transitions id="1462969919287" name="MISSING_NAME" comment="Stop" msg="">
    <preCondition id="1462969921729" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>#1462969698097</inState>
    <outState>#1462969665132</outState>
  </transitions>
  <entryPoints id="1462969665133" name="MISSING_NAME" comment="" successRequired="false" minCardinality="0" maxCardinality="2147483647">
    <task>../../../Misc/taskrepository.tsk#1225112227903</task>
    <state>#1462969665132</state>
  </entryPoints>
</alica:Plan>
