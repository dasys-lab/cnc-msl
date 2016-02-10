<?xml version="1.0" encoding="ASCII"?>
<alica:Plan xmi:version="2.0" xmlns:xmi="http://www.omg.org/XMI" xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance" xmlns:alica="http:///de.uni_kassel.vs.cn" id="1455127495970" name="TestBackroomDefence" comment="" masterPlan="false" utilityFunction="" utilityThreshold="0.1" destinationPath="Plans/Defence/Test" priority="0.0" minCardinality="0" maxCardinality="2147483647">
  <states id="1455127495971" name="Stop" comment="" entryPoint="1455127495972">
    <plans xsi:type="alica:BehaviourConfiguration">../../GenericBehaviours/Stop.beh#1413992626194</plans>
    <inTransitions>#1455127609026</inTransitions>
    <outTransitions>#1455127639286</outTransitions>
  </states>
  <states id="1455127584801" name="BackroomDefence" comment="">
    <plans xsi:type="alica:BehaviourConfiguration">../../Behaviours/BackroomDefence.beh#1454507819086</plans>
    <inTransitions>#1455127639286</inTransitions>
    <outTransitions>#1455127609026</outTransitions>
  </states>
  <transitions id="1455127609026" name="MISSING_NAME" comment="" msg="">
    <preCondition id="1455127612198" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>#1455127584801</inState>
    <outState>#1455127495971</outState>
  </transitions>
  <transitions id="1455127639286" name="MISSING_NAME" comment="" msg="">
    <preCondition id="1455127641457" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>#1455127495971</inState>
    <outState>#1455127584801</outState>
  </transitions>
  <entryPoints id="1455127495972" name="MISSING_NAME" comment="" successRequired="false" minCardinality="0" maxCardinality="2147483647">
    <task>../../../Misc/taskrepository.tsk#1225112227903</task>
    <state>#1455127495971</state>
  </entryPoints>
</alica:Plan>
