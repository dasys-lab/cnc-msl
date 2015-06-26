<?xml version="1.0" encoding="ASCII"?>
<alica:Plan xmi:version="2.0" xmlns:xmi="http://www.omg.org/XMI" xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance" xmlns:alica="http:///de.uni_kassel.vs.cn" id="1417017436952" name="ActuatorTestMaster" comment="" masterPlan="false" utilityFunction="" utilityThreshold="0.1" destinationPath="Plans/ActuatorTest" priority="0.0" minCardinality="0" maxCardinality="2147483647">
  <states id="1417017436953" name="Stop" comment="" entryPoint="1417017436954">
    <inTransitions>#1427721259649</inTransitions>
    <inTransitions>#1435321220268</inTransitions>
    <outTransitions>#1417017583911</outTransitions>
    <outTransitions>#1435321213211</outTransitions>
  </states>
  <states id="1417017460581" name="DriveForwardAndActuate" comment="">
    <plans xsi:type="alica:BehaviourConfiguration">../Behaviours/Actuate.beh#1417017552846</plans>
    <plans xsi:type="alica:BehaviourConfiguration">../Behaviours/DriveForward.beh#1417017580650</plans>
    <inTransitions>#1417017583911</inTransitions>
    <outTransitions>#1427721259649</outTransitions>
  </states>
  <states id="1435321198760" name="Joystick" comment="">
    <plans xsi:type="alica:BehaviourConfiguration">../Behaviours/Joystick.beh#1426695479346</plans>
    <inTransitions>#1435321213211</inTransitions>
    <outTransitions>#1435321220268</outTransitions>
  </states>
  <transitions id="1417017583911" name="MISSING_NAME" comment="situation start" msg="">
    <preCondition id="1417017585255" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>#1417017436953</inState>
    <outState>#1417017460581</outState>
  </transitions>
  <transitions id="1427721259649" name="MISSING_NAME" comment="situation stop" msg="">
    <preCondition id="1427721261678" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>#1417017460581</inState>
    <outState>#1417017436953</outState>
  </transitions>
  <transitions id="1435321213211" name="MISSING_NAME" comment="situtaion == joystick" msg="">
    <preCondition id="1435321214610" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>#1417017436953</inState>
    <outState>#1435321198760</outState>
  </transitions>
  <transitions id="1435321220268" name="MISSING_NAME" comment="situatio == stop" msg="">
    <preCondition id="1435321221937" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>#1435321198760</inState>
    <outState>#1417017436953</outState>
  </transitions>
  <entryPoints id="1417017436954" name="MISSING_NAME" comment="" successRequired="false" minCardinality="0" maxCardinality="2147483647">
    <task>../../Misc/taskrepository.tsk#1225112227903</task>
    <state>#1417017436953</state>
  </entryPoints>
</alica:Plan>
