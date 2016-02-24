<?xml version="1.0" encoding="ASCII"?>
<alica:Plan xmi:version="2.0" xmlns:xmi="http://www.omg.org/XMI" xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance" xmlns:alica="http:///de.uni_kassel.vs.cn" id="1450351127030" name="Move" comment="" masterPlan="false" utilityFunction="" utilityThreshold="0.1" destinationPath="Plans/Example" priority="0.0" minCardinality="0" maxCardinality="2147483647">
  <states id="1450351127031" name="Start" comment="" entryPoint="1450351127032">
    <plans xsi:type="alica:BehaviourConfiguration">NewStopbeh.beh#1449767995479</plans>
    <outTransitions>#1450446260920</outTransitions>
  </states>
  <states id="1450445779801" name="MoveTo penaltyPoint" comment="">
    <plans xsi:type="alica:BehaviourConfiguration">ThaoALignToPointWithBall.beh#1454519791394</plans>
    <inTransitions>#1450446260920</inTransitions>
  </states>
  <transitions id="1450446260920" name="MISSING_NAME" comment="" msg="">
    <preCondition id="1450446262383" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>#1450351127031</inState>
    <outState>#1450445779801</outState>
  </transitions>
  <entryPoints id="1450351127032" name="MISSING_NAME" comment="" successRequired="false" minCardinality="0" maxCardinality="2147483647">
    <task>../../Misc/taskrepository.tsk#1225112227903</task>
    <state>#1450351127031</state>
  </entryPoints>
</alica:Plan>
