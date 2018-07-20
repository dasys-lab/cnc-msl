<?xml version="1.0" encoding="ASCII"?>
<alica:Plan xmi:version="2.0" xmlns:xmi="http://www.omg.org/XMI" xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance" xmlns:alica="http:///de.uni_kassel.vs.cn" id="1489068076224" name="PointToPoint" comment="" masterPlan="true" utilityFunction="" utilityThreshold="0.1" destinationPath="Plans/TestPlans/MotorControlTest" priority="0.0" minCardinality="0" maxCardinality="2147483647">
  <states id="1489068076225" name="Stop" comment="" entryPoint="1489068076226">
    <inTransitions>#1489068108605</inTransitions>
    <outTransitions>#1489068107311</outTransitions>
  </states>
  <states id="1489068089407" name="Drive" comment="">
    <plans xsi:type="alica:BehaviourConfiguration">PointToPoint.beh#1489068194261</plans>
    <inTransitions>#1489068107311</inTransitions>
    <outTransitions>#1489068108605</outTransitions>
  </states>
  <transitions id="1489068107311" name="MISSING_NAME" comment="start signal" msg="">
    <preCondition id="1489068108429" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>#1489068076225</inState>
    <outState>#1489068089407</outState>
  </transitions>
  <transitions id="1489068108605" name="MISSING_NAME" comment="stop signal" msg="">
    <preCondition id="1489068110424" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>#1489068089407</inState>
    <outState>#1489068076225</outState>
  </transitions>
  <entryPoints id="1489068076226" name="MISSING_NAME" comment="" successRequired="false" minCardinality="0" maxCardinality="2147483647">
    <task>../../../Misc/taskrepository.tsk#1225112227903</task>
    <state>#1489068076225</state>
  </entryPoints>
</alica:Plan>
