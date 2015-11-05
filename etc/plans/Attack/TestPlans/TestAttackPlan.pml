<?xml version="1.0" encoding="ASCII"?>
<alica:Plan xmi:version="2.0" xmlns:xmi="http://www.omg.org/XMI" xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance" xmlns:alica="http:///de.uni_kassel.vs.cn" id="1436960675873" name="TestAttackPlan" comment="" masterPlan="true" utilityFunction="" utilityThreshold="0.1" destinationPath="Plans/Attack/TestPlans" priority="0.0" minCardinality="0" maxCardinality="2147483647">
  <states id="1436960675874" name="Stop" comment="" entryPoint="1436960675875">
    <plans xsi:type="alica:BehaviourConfiguration">../../GenericBehaviours/Stop.beh#1413992626194</plans>
    <inTransitions>#1436960798576</inTransitions>
    <outTransitions>#1436960796936</outTransitions>
  </states>
  <states id="1436960747966" name="Start" comment="">
    <plans xsi:type="alica:Plan">AttackOppGoalPlan.pml#1437902649389</plans>
    <inTransitions>#1436960796936</inTransitions>
    <outTransitions>#1436960798576</outTransitions>
  </states>
  <transitions id="1436960796936" name="MISSING_NAME" comment="Situation == start" msg="">
    <preCondition id="1436960798234" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>#1436960675874</inState>
    <outState>#1436960747966</outState>
  </transitions>
  <transitions id="1436960798576" name="MISSING_NAME" comment="situation == stop || anyChildSucc" msg="">
    <preCondition id="1436960799378" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>#1436960747966</inState>
    <outState>#1436960675874</outState>
  </transitions>
  <entryPoints id="1436960675875" name="MISSING_NAME" comment="" successRequired="false" minCardinality="0" maxCardinality="2147483647">
    <task>../../../Misc/taskrepository.tsk#1225112227903</task>
    <state>#1436960675874</state>
  </entryPoints>
</alica:Plan>
