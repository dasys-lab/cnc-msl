<?xml version="1.0" encoding="ASCII"?>
<alica:Plan xmi:version="2.0" xmlns:xmi="http://www.omg.org/XMI" xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance" xmlns:alica="http:///de.uni_kassel.vs.cn" id="1467309055366" name="ActuatorPassTest" comment="" masterPlan="false" utilityFunction="" utilityThreshold="0.1" destinationPath="Plans/Attack/TestPlans" priority="0.0" minCardinality="0" maxCardinality="2147483647">
  <states id="1467309055367" name="Stop" comment="" entryPoint="1467309055368">
    <plans xsi:type="alica:BehaviourConfiguration">../../GenericBehaviours/Stop.beh#1413992626194</plans>
    <inTransitions>#1467309132307</inTransitions>
    <outTransitions>#1467309114647</outTransitions>
  </states>
  <states id="1467309076558" name="ReceivePass" comment="">
    <plans xsi:type="alica:BehaviourConfiguration">../../Behaviours/ActuatorPassTest.beh#1467309206305</plans>
    <inTransitions>#1467309114647</inTransitions>
    <outTransitions>#1467309132307</outTransitions>
  </states>
  <transitions id="1467309114647" name="MISSING_NAME" comment="start" msg="">
    <preCondition id="1467309116799" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>#1467309055367</inState>
    <outState>#1467309076558</outState>
  </transitions>
  <transitions id="1467309132307" name="MISSING_NAME" comment="stop" msg="">
    <preCondition id="1467309134723" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>#1467309076558</inState>
    <outState>#1467309055367</outState>
  </transitions>
  <entryPoints id="1467309055368" name="MISSING_NAME" comment="" successRequired="false" minCardinality="0" maxCardinality="2147483647">
    <task>../../../Misc/taskrepository.tsk#1225112227903</task>
    <state>#1467309055367</state>
  </entryPoints>
</alica:Plan>
