<?xml version="1.0" encoding="ASCII"?>
<alica:Plan xmi:version="2.0" xmlns:xmi="http://www.omg.org/XMI" xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance" xmlns:alica="http:///de.uni_kassel.vs.cn" id="1518256493210" name="TestActuateMaster" comment="" masterPlan="false" utilityFunction="" utilityThreshold="0.1" destinationPath="Plans/Standards/Own/Test" priority="0.0" minCardinality="0" maxCardinality="2147483647">
  <states id="1518256493211" name="Stop" comment="" entryPoint="1518256493212">
    <plans xsi:type="alica:BehaviourConfiguration">../../../GenericBehaviours/Stop.beh#1413992626194</plans>
    <inTransitions>#1518259441896</inTransitions>
    <outTransitions>#1518259433380</outTransitions>
  </states>
  <states id="1518259419841" name="TestActuate" comment="">
    <plans xsi:type="alica:Plan">TestConfigurableActuate.pml#1518256475491</plans>
    <inTransitions>#1518259433380</inTransitions>
    <outTransitions>#1518259441896</outTransitions>
  </states>
  <transitions id="1518259433380" name="MISSING_NAME" comment="situation start" msg="">
    <preCondition id="1518259434540" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>#1518256493211</inState>
    <outState>#1518259419841</outState>
  </transitions>
  <transitions id="1518259441896" name="MISSING_NAME" comment="situation stop" msg="">
    <preCondition id="1518259443119" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>#1518259419841</inState>
    <outState>#1518256493211</outState>
  </transitions>
  <entryPoints id="1518256493212" name="MISSING_NAME" comment="" successRequired="false" minCardinality="0" maxCardinality="2147483647">
    <task>../../../../Misc/taskrepository.tsk#1225112227903</task>
    <state>#1518256493211</state>
  </entryPoints>
</alica:Plan>
