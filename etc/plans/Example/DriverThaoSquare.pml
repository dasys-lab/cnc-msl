<?xml version="1.0" encoding="ASCII"?>
<alica:Plan xmi:version="2.0" xmlns:xmi="http://www.omg.org/XMI" xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance" xmlns:alica="http:///de.uni_kassel.vs.cn" id="1450269525655" name="DriverThaoSquare" comment="" masterPlan="false" utilityFunction="" utilityThreshold="0.1" destinationPath="Plans/Example" priority="0.0" minCardinality="0" maxCardinality="2147483647">
  <states id="1450269525656" name="Start" comment="" entryPoint="1450269525657">
    <plans xsi:type="alica:BehaviourConfiguration">NewStopbeh.beh#1449767995479</plans>
    <outTransitions>#1450269556583</outTransitions>
  </states>
  <states id="1450269533144" name="MoveSquare" comment="">
    <plans xsi:type="alica:BehaviourConfiguration">MoveThaoSquarebeh.beh#1450269596860</plans>
    <inTransitions>#1450269556583</inTransitions>
  </states>
  <transitions id="1450269556583" name="MISSING_NAME" comment="" msg="">
    <preCondition id="1450269558021" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>#1450269525656</inState>
    <outState>#1450269533144</outState>
  </transitions>
  <entryPoints id="1450269525657" name="MISSING_NAME" comment="" successRequired="false" minCardinality="0" maxCardinality="2147483647">
    <task>../../Misc/taskrepository.tsk#1225112227903</task>
    <state>#1450269525656</state>
  </entryPoints>
</alica:Plan>
