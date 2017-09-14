<?xml version="1.0" encoding="ASCII"?>
<alica:Plan xmi:version="2.0" xmlns:xmi="http://www.omg.org/XMI" xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance" xmlns:alica="http:///de.uni_kassel.vs.cn" id="1492620432380" name="TestRotation" comment="" masterPlan="false" utilityFunction="" utilityThreshold="0.1" destinationPath="Plans/Calibration" priority="0.0" minCardinality="0" maxCardinality="2147483647">
  <states id="1492620432381" name="NewState" comment="" entryPoint="1492620432383">
    <plans xsi:type="alica:BehaviourConfiguration">TestRotationRotation.beh#1492620523847</plans>
    <outTransitions>#1492620539509</outTransitions>
  </states>
  <states id="1492620463395" name="Stop" comment="">
    <plans xsi:type="alica:BehaviourConfiguration">../GenericBehaviours/Stop.beh#1413992626194</plans>
    <inTransitions>#1492620539509</inTransitions>
  </states>
  <transitions id="1492620539509" name="MISSING_NAME" comment="success" msg="">
    <preCondition id="1492620542706" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>#1492620432381</inState>
    <outState>#1492620463395</outState>
  </transitions>
  <entryPoints id="1492620432383" name="MISSING_NAME" comment="" successRequired="false" minCardinality="0" maxCardinality="2147483647">
    <task>../../Misc/taskrepository.tsk#1225112227903</task>
    <state>#1492620432381</state>
  </entryPoints>
</alica:Plan>
