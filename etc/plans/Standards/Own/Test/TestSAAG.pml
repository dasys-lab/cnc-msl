<?xml version="1.0" encoding="ASCII"?>
<alica:Plan xmi:version="2.0" xmlns:xmi="http://www.omg.org/XMI" xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance" xmlns:alica="http:///de.uni_kassel.vs.cn" id="1516967282723" name="TestSAAG" comment="" masterPlan="false" utilityFunction="" utilityThreshold="0.1" destinationPath="Plans/Standards/Own/Test" priority="0.0" minCardinality="2" maxCardinality="2">
  <states id="1516967282724" name="NewState" comment="" entryPoint="1516967282725">
    <plans xsi:type="alica:BehaviourConfiguration">../../../GenericStandards/StandardAlignAndGrab.beh#1467436134025</plans>
    <plans xsi:type="alica:BehaviourConfiguration">../../../TestPlans/DribbleControlTest/DribbleControlMOS.beh#1479905216821</plans>
    <outTransitions>#1516967326504</outTransitions>
  </states>
  <states id="1516967308197" name="Success" comment="">
    <plans xsi:type="alica:BehaviourConfiguration">../../../GenericBehaviours/Stop.beh#1413992626194</plans>
    <inTransitions>#1516967326504</inTransitions>
  </states>
  <states id="1516967376299" name="Receive" comment="" entryPoint="1516967365651">
    <plans xsi:type="alica:BehaviourConfiguration">../../../GenericBehaviours/Stop.beh#1413992626194</plans>
  </states>
  <transitions id="1516967326504" name="MISSING_NAME" comment="" msg="">
    <preCondition id="1516967327649" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>#1516967282724</inState>
    <outState>#1516967308197</outState>
  </transitions>
  <entryPoints id="1516967282725" name="MISSING_NAME" comment="" successRequired="false" minCardinality="1" maxCardinality="1">
    <task>../../../../Misc/taskrepository.tsk#1225112227903</task>
    <state>#1516967282724</state>
  </entryPoints>
  <entryPoints id="1516967365651" name="ReceiveStandard" comment="" successRequired="false" minCardinality="1" maxCardinality="1">
    <task>../../../../Misc/taskrepository.tsk#1439997023446</task>
    <state>#1516967376299</state>
  </entryPoints>
</alica:Plan>
