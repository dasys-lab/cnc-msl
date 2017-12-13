<?xml version="1.0" encoding="ASCII"?>
<alica:Plan xmi:version="2.0" xmlns:xmi="http://www.omg.org/XMI" xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance" xmlns:alica="http:///de.uni_kassel.vs.cn" id="1513182639097" name="DribbleTestMaster" comment="" masterPlan="true" utilityFunction="" utilityThreshold="0.1" destinationPath="" priority="0.0" minCardinality="0" maxCardinality="2147483647">
  <states id="1513182639098" name="Stop" comment="" entryPoint="1513182639099">
    <plans xsi:type="alica:BehaviourConfiguration">../GenericBehaviours/Stop.beh#1413992626194</plans>
    <inTransitions>#1513182712911</inTransitions>
    <outTransitions>#1513182711121</outTransitions>
  </states>
  <states id="1513182703528" name="Start" comment="">
    <plans xsi:type="alica:Plan">WayPointPlan.pml#1513182751590</plans>
    <inTransitions>#1513182711121</inTransitions>
    <outTransitions>#1513182712911</outTransitions>
  </states>
  <transitions id="1513182711121" name="MISSING_NAME" comment="Stop to Start" msg="">
    <preCondition id="1513182712750" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>#1513182639098</inState>
    <outState>#1513182703528</outState>
  </transitions>
  <transitions id="1513182712911" name="MISSING_NAME" comment="Start to Stop" msg="">
    <preCondition id="1513182715236" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>#1513182703528</inState>
    <outState>#1513182639098</outState>
  </transitions>
  <entryPoints id="1513182639099" name="MISSING_NAME" comment="" successRequired="false" minCardinality="0" maxCardinality="2147483647">
    <task>../../Misc/taskrepository.tsk#1225112227903</task>
    <state>#1513182639098</state>
  </entryPoints>
</alica:Plan>
