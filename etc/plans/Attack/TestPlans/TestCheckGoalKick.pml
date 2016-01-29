<?xml version="1.0" encoding="ASCII"?>
<alica:Plan xmi:version="2.0" xmlns:xmi="http://www.omg.org/XMI" xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance" xmlns:alica="http:///de.uni_kassel.vs.cn" id="1449076138236" name="TestCheckGoalKick" comment="" masterPlan="false" utilityFunction="" utilityThreshold="0.1" destinationPath="Plans/Attack/TestPlans" priority="0.0" minCardinality="0" maxCardinality="2147483647">
  <states id="1449076138237" name="Stop" comment="" entryPoint="1449076138238">
    <plans xsi:type="alica:BehaviourConfiguration">../../GenericBehaviours/Stop.beh#1413992626194</plans>
    <inTransitions>#1449076203977</inTransitions>
    <inTransitions>#1453918062691</inTransitions>
    <outTransitions>#1449076202114</outTransitions>
    <outTransitions>#1453918060467</outTransitions>
  </states>
  <states id="1449076173801" name="CheckGoalKick" comment="">
    <plans xsi:type="alica:BehaviourConfiguration">../../GenericBehaviours/CheckGoalKick.beh#1449076029919</plans>
    <inTransitions>#1449076202114</inTransitions>
    <outTransitions>#1449076203977</outTransitions>
  </states>
  <states id="1453918051237" name="Joystick" comment="">
    <plans xsi:type="alica:BehaviourConfiguration">../../Behaviours/Joystick.beh#1421854995808</plans>
    <inTransitions>#1453918060467</inTransitions>
    <outTransitions>#1453918062691</outTransitions>
  </states>
  <transitions id="1449076202114" name="MISSING_NAME" comment="Start" msg="">
    <preCondition id="1449076203800" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>#1449076138237</inState>
    <outState>#1449076173801</outState>
  </transitions>
  <transitions id="1449076203977" name="MISSING_NAME" comment="Stop" msg="">
    <preCondition id="1449076205925" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>#1449076173801</inState>
    <outState>#1449076138237</outState>
  </transitions>
  <transitions id="1453918060467" name="MISSING_NAME" comment="" msg="">
    <preCondition id="1453918062457" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>#1449076138237</inState>
    <outState>#1453918051237</outState>
  </transitions>
  <transitions id="1453918062691" name="MISSING_NAME" comment="" msg="">
    <preCondition id="1453918066476" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>#1453918051237</inState>
    <outState>#1449076138237</outState>
  </transitions>
  <entryPoints id="1449076138238" name="MISSING_NAME" comment="" successRequired="false" minCardinality="0" maxCardinality="2147483647">
    <task>../../../Misc/taskrepository.tsk#1225112227903</task>
    <state>#1449076138237</state>
  </entryPoints>
</alica:Plan>
