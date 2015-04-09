<?xml version="1.0" encoding="ASCII"?>
<alica:Plan xmi:version="2.0" xmlns:xmi="http://www.omg.org/XMI" xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance" xmlns:alica="http:///de.uni_kassel.vs.cn" id="1428508782222" name="GenericOwnStandardExecution" comment="" masterPlan="false" utilityFunction="" utilityThreshold="0.1" destinationPath="Plans/GameStrategy/OwnStandards" priority="0.0" minCardinality="0" maxCardinality="2147483647">
  <states id="1428508810441" name="ExecuterExecution" comment="" entryPoint="1428508810442">
    <plans xsi:type="alica:BehaviourConfiguration">../../GenericBehaviours/StandardAlignAndShoot.beh#1428509031167</plans>
    <outTransitions>#1428509247624</outTransitions>
  </states>
  <states id="1428508844911" name="ReceiverExecution" comment="" entryPoint="1428508819333">
    <plans xsi:type="alica:BehaviourConfiguration">../../GenericBehaviours/Stop.beh#1413992626194</plans>
    <outTransitions>#1428509280543</outTransitions>
  </states>
  <states id="1428508847088" name="StandardDefendExecution" comment="" entryPoint="1428508821680">
    <plans xsi:type="alica:BehaviourConfiguration">../../GenericBehaviours/Stop.beh#1413992626194</plans>
  </states>
  <states xsi:type="alica:SuccessState" id="1428509108972" name="NewSuccessState" comment="">
    <inTransitions>#1428509286818</inTransitions>
  </states>
  <states id="1428509131566" name="Block" comment="">
    <plans xsi:type="alica:BehaviourConfiguration">../../GenericBehaviours/Stop.beh#1413992626194</plans>
    <inTransitions>#1428509247624</inTransitions>
  </states>
  <states id="1428509271725" name="Receive" comment="">
    <plans xsi:type="alica:BehaviourConfiguration">../../GenericBehaviours/StandardReceive.beh#1428509534191</plans>
    <inTransitions>#1428509280543</inTransitions>
    <outTransitions>#1428509286818</outTransitions>
  </states>
  <transitions id="1428509247624" name="MISSING_NAME" comment="anyChildSuccess" msg="">
    <preCondition id="1428509248757" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>#1428508810441</inState>
    <outState>#1428509131566</outState>
  </transitions>
  <transitions id="1428509280543" name="MISSING_NAME" comment="one robot in state block:1428509131566" msg="">
    <preCondition id="1428509282512" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>#1428508844911</inState>
    <outState>#1428509271725</outState>
  </transitions>
  <transitions id="1428509286818" name="MISSING_NAME" comment="anyChildSuccess" msg="">
    <preCondition id="1428509288287" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>#1428509271725</inState>
    <outState>#1428509108972</outState>
  </transitions>
  <entryPoints id="1428508810442" name="Executer" comment="" successRequired="false" minCardinality="0" maxCardinality="2147483647">
    <task>../../../Misc/taskrepository.tsk#1428507765874</task>
    <state>#1428508810441</state>
  </entryPoints>
  <entryPoints id="1428508819333" name="Receiver" comment="" successRequired="true" minCardinality="0" maxCardinality="2147483647">
    <task>../../../Misc/taskrepository.tsk#1428507797481</task>
    <state>#1428508844911</state>
  </entryPoints>
  <entryPoints id="1428508821680" name="StandardDefend" comment="" successRequired="false" minCardinality="0" maxCardinality="2147483647">
    <task>../../../Misc/taskrepository.tsk#1225112227903</task>
    <state>#1428508847088</state>
  </entryPoints>
</alica:Plan>
