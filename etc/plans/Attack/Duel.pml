<?xml version="1.0" encoding="ASCII"?>
<alica:Plan xmi:version="2.0" xmlns:xmi="http://www.omg.org/XMI" xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance" xmlns:alica="http:///de.uni_kassel.vs.cn" id="1450178655416" name="Duel" comment="" masterPlan="false" utilityFunction="" utilityThreshold="0.1" destinationPath="Plans/Attack" priority="0.0" minCardinality="1" maxCardinality="2147483647">
  <states id="1458127402884" name="Duel" comment="" entryPoint="1450178655418">
    <plans xsi:type="alica:BehaviourConfiguration">../Behaviours/Duel.beh#1450178707835</plans>
    <outTransitions>#1458131557622</outTransitions>
  </states>
  <states xsi:type="alica:SuccessState" id="1458131546373" name="Success" comment="">
    <inTransitions>#1458131557622</inTransitions>
  </states>
  <states id="1458662648729" name="chill" comment="" entryPoint="1458662674153">
    <plans xsi:type="alica:BehaviourConfiguration">../GenericBehaviours/Stop.beh#1413992626194</plans>
  </states>
  <transitions id="1458131557622" name="MISSING_NAME" comment="success" msg="">
    <preCondition id="1458131561310" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>#1458127402884</inState>
    <outState>#1458131546373</outState>
  </transitions>
  <entryPoints id="1450178655418" name="MISSING_NAME" comment="" successRequired="true" minCardinality="1" maxCardinality="1">
    <task>../../Misc/taskrepository.tsk#1225112227903</task>
    <state>#1458127402884</state>
  </entryPoints>
  <entryPoints id="1458662674153" name="NewEntryPoint" comment="" successRequired="false" minCardinality="0" maxCardinality="2147483647">
    <task>../../Misc/taskrepository.tsk#1225112227903</task>
    <state>#1458662648729</state>
  </entryPoints>
</alica:Plan>
