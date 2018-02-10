<?xml version="1.0" encoding="ASCII"?>
<alica:Plan xmi:version="2.0" xmlns:xmi="http://www.omg.org/XMI" xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance" xmlns:alica="http:///de.uni_kassel.vs.cn" id="1518256475491" name="TestConfigurableActuate" comment="" masterPlan="false" utilityFunction="" utilityThreshold="0.1" destinationPath="Plans/Standards/Own/Test" priority="0.0" minCardinality="2" maxCardinality="2">
  <states id="1518256475492" name="UseConfig" comment="" entryPoint="1518256475493">
    <plans xsi:type="alica:BehaviourConfiguration">../../../GenericStandards/StandardActuate.beh#1435766278023</plans>
    <outTransitions>#1518257017363</outTransitions>
  </states>
  <states id="1518256723974" name="UseParam" comment="" entryPoint="1518256719934">
    <plans xsi:type="alica:BehaviourConfiguration">../../../GenericStandards/StandardActuate.beh#1518257626027</plans>
    <outTransitions>#1518257015951</outTransitions>
  </states>
  <states id="1518257009829" name="UseParam" comment="">
    <plans xsi:type="alica:BehaviourConfiguration">../../../GenericStandards/StandardActuate.beh#1518257626027</plans>
    <inTransitions>#1518257017363</inTransitions>
    <outTransitions>#1518257030924</outTransitions>
  </states>
  <states id="1518257013318" name="UseConfig" comment="">
    <plans xsi:type="alica:BehaviourConfiguration">../../../GenericStandards/StandardActuate.beh#1435766278023</plans>
    <inTransitions>#1518257015951</inTransitions>
    <outTransitions>#1518257035564</outTransitions>
  </states>
  <states id="1518257028132" name="UseConfig" comment="">
    <plans xsi:type="alica:BehaviourConfiguration">../../../GenericStandards/StandardActuate.beh#1435766278023</plans>
    <inTransitions>#1518257030924</inTransitions>
  </states>
  <states id="1518257033220" name="UseParam" comment="">
    <plans xsi:type="alica:BehaviourConfiguration">../../../GenericStandards/StandardActuate.beh#1518257157495</plans>
    <inTransitions>#1518257035564</inTransitions>
  </states>
  <transitions id="1518257015951" name="MISSING_NAME" comment="" msg="">
    <preCondition id="1518257017227" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>#1518256723974</inState>
    <outState>#1518257013318</outState>
  </transitions>
  <transitions id="1518257017363" name="MISSING_NAME" comment="" msg="">
    <preCondition id="1518257026137" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>#1518256475492</inState>
    <outState>#1518257009829</outState>
  </transitions>
  <transitions id="1518257030924" name="MISSING_NAME" comment="" msg="">
    <preCondition id="1518257031921" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>#1518257009829</inState>
    <outState>#1518257028132</outState>
  </transitions>
  <transitions id="1518257035564" name="MISSING_NAME" comment="" msg="">
    <preCondition id="1518257036260" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>#1518257013318</inState>
    <outState>#1518257033220</outState>
  </transitions>
  <entryPoints id="1518256475493" name="MISSING_NAME" comment="" successRequired="false" minCardinality="1" maxCardinality="1">
    <task>../../../../Misc/taskrepository.tsk#1225112227903</task>
    <state>#1518256475492</state>
  </entryPoints>
  <entryPoints id="1518256719934" name="NewEntryPoint" comment="" successRequired="false" minCardinality="1" maxCardinality="1">
    <task>../../../../Misc/taskrepository.tsk#1225112227903</task>
    <state>#1518256723974</state>
  </entryPoints>
</alica:Plan>
