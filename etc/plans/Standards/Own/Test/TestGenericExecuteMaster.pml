<?xml version="1.0" encoding="ASCII"?>
<alica:Plan xmi:version="2.0" xmlns:xmi="http://www.omg.org/XMI" xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance" xmlns:alica="http:///de.uni_kassel.vs.cn" id="1513602784836" name="TestGenericExecuteMaster" comment="" masterPlan="true" utilityFunction="" utilityThreshold="0.1" destinationPath="Plans/Standards/Own/Test" priority="0.0" minCardinality="0" maxCardinality="2147483647">
  <states id="1513602784837" name="Stop" comment="" entryPoint="1513602784838">
    <plans xsi:type="alica:BehaviourConfiguration">../../../GenericBehaviours/Stop.beh#1413992626194</plans>
    <inTransitions>#1513602936810</inTransitions>
    <outTransitions>#1513602923359</outTransitions>
  </states>
  <states id="1513602904558" name="GenEx" comment="">
    <plans xsi:type="alica:Plan">../../../GenericStandards/GenericExecute.pml#1431522123418</plans>
    <inTransitions>#1513602923359</inTransitions>
    <outTransitions>#1513602936810</outTransitions>
  </states>
  <transitions id="1513602923359" name="MISSING_NAME" comment="situation stop" msg="">
    <preCondition id="1513602924281" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>#1513602784837</inState>
    <outState>#1513602904558</outState>
  </transitions>
  <transitions id="1513602936810" name="MISSING_NAME" comment="situation start" msg="">
    <preCondition id="1513602937857" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>#1513602904558</inState>
    <outState>#1513602784837</outState>
  </transitions>
  <entryPoints id="1513602784838" name="MISSING_NAME" comment="" successRequired="false" minCardinality="0" maxCardinality="2147483647">
    <task>../../../../Misc/taskrepository.tsk#1225112227903</task>
    <state>#1513602784837</state>
  </entryPoints>
</alica:Plan>
