<?xml version="1.0" encoding="ASCII"?>
<alica:Plan xmi:version="2.0" xmlns:xmi="http://www.omg.org/XMI" xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance" xmlns:alica="http:///de.uni_kassel.vs.cn" id="1462969665131" name="RobotMovementTestPlan" comment="" masterPlan="true" utilityFunction="" utilityThreshold="0.1" destinationPath="Plans/Attack/TestPlans" priority="0.0" minCardinality="0" maxCardinality="2147483647">
  <states id="1462969665132" name="Stop" comment="" entryPoint="1462969665133">
    <plans xsi:type="alica:BehaviourConfiguration">../../GenericBehaviours/Stop.beh#1413992626194</plans>
    <inTransitions>#1462969786619</inTransitions>
    <outTransitions>#1462969759156</outTransitions>
  </states>
  <states id="1462969884284" name="dribbleTest" comment="">
    <plans xsi:type="alica:BehaviourConfiguration">../../Behaviours/RobotMovementDribbleTest.beh#1462969753310</plans>
    <plans xsi:type="alica:BehaviourConfiguration">../../TestPlans/DribbleControlTest/DribbleControlMOS.beh#1479905216821</plans>
    <inTransitions>#1462969759156</inTransitions>
    <outTransitions>#1462969786619</outTransitions>
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
  <entryPoints id="1462969665133" name="MISSING_NAME" comment="" successRequired="false" minCardinality="0" maxCardinality="2147483647">
    <task>../../../Misc/taskrepository.tsk#1225112227903</task>
    <state>#1462969665132</state>
  </entryPoints>
</alica:Plan>
