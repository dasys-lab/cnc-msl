<?xml version="1.0" encoding="ASCII"?>
<alica:Plan xmi:version="2.0" xmlns:xmi="http://www.omg.org/XMI" xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance" xmlns:alica="http:///de.uni_kassel.vs.cn" id="1518866346673" name="DribbleTestMaster" comment="" masterPlan="true" utilityFunction="" utilityThreshold="0.1" destinationPath="Plans/TestPlans/DribbleTestMOS" priority="0.0" minCardinality="0" maxCardinality="2147483647">
  <states id="1518866346674" name="Stop" comment="" entryPoint="1518866346675">
    <plans xsi:type="alica:BehaviourConfiguration">../../GenericBehaviours/Stop.beh#1413992626194</plans>
    <inTransitions>#1518866390610</inTransitions>
    <outTransitions>#1518866388604</outTransitions>
  </states>
  <states id="1518866360499" name="Start" comment="">
    <plans xsi:type="alica:Plan">TestDribbleMOS.pml#1518622787399</plans>
    <plans xsi:type="alica:BehaviourConfiguration">../DribbleControlTest/DribbleControlMOS.beh#1479905216821</plans>
    <inTransitions>#1518866388604</inTransitions>
    <outTransitions>#1518866390610</outTransitions>
  </states>
  <transitions id="1518866388604" name="MISSING_NAME" comment="start pressed" msg="">
    <preCondition id="1518866390521" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>#1518866346674</inState>
    <outState>#1518866360499</outState>
  </transitions>
  <transitions id="1518866390610" name="MISSING_NAME" comment="stop pressed" msg="">
    <preCondition id="1518866391883" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>#1518866360499</inState>
    <outState>#1518866346674</outState>
  </transitions>
  <entryPoints id="1518866346675" name="MISSING_NAME" comment="" successRequired="false" minCardinality="0" maxCardinality="2147483647">
    <task>../../../Misc/taskrepository.tsk#1225112227903</task>
    <state>#1518866346674</state>
  </entryPoints>
</alica:Plan>
