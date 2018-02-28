<?xml version="1.0" encoding="ASCII"?>
<alica:Plan xmi:version="2.0" xmlns:xmi="http://www.omg.org/XMI" xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance" xmlns:alica="http:///de.uni_kassel.vs.cn" id="1498663554104" name="TestDribbleAroundTheBall" comment="" masterPlan="true" utilityFunction="" utilityThreshold="0.1" destinationPath="Plans/TestPlans/DribbleControlTest" priority="0.0" minCardinality="0" maxCardinality="2147483647">
  <states id="1498663554105" name="Stop" comment="" entryPoint="1498663554106">
    <plans xsi:type="alica:BehaviourConfiguration">../../GenericBehaviours/Stop.beh#1413992626194</plans>
    <inTransitions>#1498664098783</inTransitions>
    <outTransitions>#1498664085559</outTransitions>
  </states>
  <states id="1498663949107" name="Dribble" comment="">
    <plans xsi:type="alica:Plan">TestDribble.pml#1498664114905</plans>
    <inTransitions>#1498664085559</inTransitions>
    <outTransitions>#1498664098783</outTransitions>
  </states>
  <transitions id="1498664085559" name="MISSING_NAME" comment="Start" msg="">
    <preCondition id="1498664087062" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>#1498663554105</inState>
    <outState>#1498663949107</outState>
  </transitions>
  <transitions id="1498664098783" name="MISSING_NAME" comment="Stop" msg="">
    <preCondition id="1498664100103" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>#1498663949107</inState>
    <outState>#1498663554105</outState>
  </transitions>
  <entryPoints id="1498663554106" name="MISSING_NAME" comment="" successRequired="false" minCardinality="0" maxCardinality="2147483647">
    <task>../../../Misc/taskrepository.tsk#1225112227903</task>
    <state>#1498663554105</state>
  </entryPoints>
</alica:Plan>
