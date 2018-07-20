<?xml version="1.0" encoding="ASCII"?>
<alica:Plan xmi:version="2.0" xmlns:xmi="http://www.omg.org/XMI" xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance" xmlns:alica="http:///de.uni_kassel.vs.cn" id="1457698586746" name="KickCurveTuning" comment="" masterPlan="true" utilityFunction="" utilityThreshold="0.1" destinationPath="Plans/TestPlans/KickCurveTuning" priority="0.0" minCardinality="0" maxCardinality="2147483647">
  <states id="1457698586747" name="Stop" comment="" entryPoint="1457698586748">
    <plans xsi:type="alica:BehaviourConfiguration">../../GenericBehaviours/Stop.beh#1413992626194</plans>
    <inTransitions>#1457698634760</inTransitions>
    <outTransitions>#1457698633209</outTransitions>
  </states>
  <states id="1457698608390" name="LaserBallTracking" comment="">
    <plans xsi:type="alica:BehaviourConfiguration">LaserBallTracking.beh#1457698689219</plans>
    <inTransitions>#1457698633209</inTransitions>
    <outTransitions>#1457698634760</outTransitions>
  </states>
  <transitions id="1457698633209" name="MISSING_NAME" comment="situation=start" msg="">
    <preCondition id="1457698634601" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>#1457698586747</inState>
    <outState>#1457698608390</outState>
  </transitions>
  <transitions id="1457698634760" name="MISSING_NAME" comment="situation=stop" msg="">
    <preCondition id="1457698635818" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>#1457698608390</inState>
    <outState>#1457698586747</outState>
  </transitions>
  <entryPoints id="1457698586748" name="MISSING_NAME" comment="" successRequired="false" minCardinality="0" maxCardinality="2147483647">
    <task>../../../Misc/taskrepository.tsk#1225112227903</task>
    <state>#1457698586747</state>
  </entryPoints>
</alica:Plan>
