<?xml version="1.0" encoding="ASCII"?>
<alica:Plan xmi:version="2.0" xmlns:xmi="http://www.omg.org/XMI" xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance" xmlns:alica="http:///de.uni_kassel.vs.cn" id="1433938652021" name="ExamplePlan" comment="" masterPlan="true" utilityFunction="" utilityThreshold="0.1" destinationPath="Plans/Example" priority="0.0" minCardinality="0" maxCardinality="2147483647">
  <states id="1433938970029" name="Wait" comment="" entryPoint="1433938652023">
    <plans xsi:type="alica:BehaviourConfiguration">NewStopbeh.beh#1449767995479</plans>
    <inTransitions>#1433939023953</inTransitions>
    <outTransitions>#1433939022457</outTransitions>
  </states>
  <states id="1433938974495" name="DriveInSquare" comment="">
    <plans xsi:type="alica:BehaviourConfiguration">../Behaviours/StdExecutorGrabBall.beh#1441209089978</plans>
    <inTransitions>#1433939022457</inTransitions>
    <outTransitions>#1433939023953</outTransitions>
  </states>
  <transitions id="1433939022457" name="MISSING_NAME" comment="start signal received" msg="">
    <preCondition id="1433939023616" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>#1433938970029</inState>
    <outState>#1433938974495</outState>
  </transitions>
  <transitions id="1433939023953" name="MISSING_NAME" comment="stop signal received" msg="">
    <preCondition id="1433939026572" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>#1433938974495</inState>
    <outState>#1433938970029</outState>
  </transitions>
  <entryPoints id="1433938652023" name="MISSING_NAME" comment="" successRequired="false" minCardinality="0" maxCardinality="2147483647">
    <task>../../Misc/taskrepository.tsk#1225112227903</task>
    <state>#1433938970029</state>
  </entryPoints>
</alica:Plan>
