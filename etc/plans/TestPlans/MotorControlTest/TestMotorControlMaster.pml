<?xml version="1.0" encoding="ASCII"?>
<alica:Plan xmi:version="2.0" xmlns:xmi="http://www.omg.org/XMI" xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance" xmlns:alica="http:///de.uni_kassel.vs.cn" id="1482163489121" name="TestMotorControlMaster" comment="" destinationPath="Plans/TestPlans/MotorControlTest" priority="0.0" minCardinality="0" maxCardinality="2147483647" masterPlan="false" utilityFunction="" utilityThreshold="0.1">
  <states id="1482163489122" name="Stop" comment="" entryPoint="1482163489123">
    <plans xsi:type="alica:Behaviour">GenericBehaviours/Stop.beh#1413992604875</plans>
    <inTransitions>TestPlans/MotorControlTest/TestMotorControlMaster.pml#1482163828829</inTransitions>
    <outTransitions>TestPlans/MotorControlTest/TestMotorControlMaster.pml#1482163826573</outTransitions>
  </states>
  <states id="1482163786868" name="MotorControl" comment="">
    <plans xsi:type="alica:Behaviour">TestPlans/MotorControlTest/TestMotorControl.beh#1482163964536</plans>
    <inTransitions>TestPlans/MotorControlTest/TestMotorControlMaster.pml#1482163826573</inTransitions>
    <outTransitions>TestPlans/MotorControlTest/TestMotorControlMaster.pml#1482163828829</outTransitions>
  </states>
  <transitions id="1482163826573" name="MISSING_NAME" comment="" msg="">
    <preCondition id="1482163828612" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>TestPlans/MotorControlTest/TestMotorControlMaster.pml#1482163489122</inState>
    <outState>TestPlans/MotorControlTest/TestMotorControlMaster.pml#1482163786868</outState>
  </transitions>
  <transitions id="1482163828829" name="MISSING_NAME" comment="" msg="">
    <preCondition id="1482163831043" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>TestPlans/MotorControlTest/TestMotorControlMaster.pml#1482163786868</inState>
    <outState>TestPlans/MotorControlTest/TestMotorControlMaster.pml#1482163489122</outState>
  </transitions>
  <entryPoints id="1482163489123" name="MISSING_NAME" comment="" successRequired="false" minCardinality="0" maxCardinality="2147483647">
    <task>taskrepository.tsk#1225112227903</task>
    <state>TestPlans/MotorControlTest/TestMotorControlMaster.pml#1482163489122</state>
  </entryPoints>
</alica:Plan>
