<?xml version="1.0" encoding="ASCII"?>
<alica:Plan xmi:version="2.0" xmlns:xmi="http://www.omg.org/XMI" xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance" xmlns:alica="http:///de.uni_kassel.vs.cn" id="1473862711641" name="RobotMovementTest" comment="" destinationPath="Plans/TestPlans/RobotMovement" priority="0.0" minCardinality="0" maxCardinality="2147483647" masterPlan="false" utilityFunction="" utilityThreshold="0.1">
  <states id="1473862711642" name="Stop" comment="" entryPoint="1473862711643">
    <inTransitions>TestPlans/RobotMovement/RobotMovementTest.pml#1473865277226</inTransitions>
    <outTransitions>TestPlans/RobotMovement/RobotMovementTest.pml#1473862896106</outTransitions>
  </states>
  <states id="1473862820126" name="Test" comment="">
    <plans xsi:type="alica:Behaviour">TestPlans/RobotMovement/Behaviours/RobotMovementTest.beh#1473862842303</plans>
    <inTransitions>TestPlans/RobotMovement/RobotMovementTest.pml#1473862896106</inTransitions>
    <outTransitions>TestPlans/RobotMovement/RobotMovementTest.pml#1473865277226</outTransitions>
  </states>
  <transitions id="1473862896106" name="MISSING_NAME" comment="start" msg="">
    <preCondition id="1473862897803" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>TestPlans/RobotMovement/RobotMovementTest.pml#1473862711642</inState>
    <outState>TestPlans/RobotMovement/RobotMovementTest.pml#1473862820126</outState>
  </transitions>
  <transitions id="1473865277226" name="MISSING_NAME" comment="Stop" msg="">
    <preCondition id="1473865278666" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>TestPlans/RobotMovement/RobotMovementTest.pml#1473862820126</inState>
    <outState>TestPlans/RobotMovement/RobotMovementTest.pml#1473862711642</outState>
  </transitions>
  <entryPoints id="1473862711643" name="MISSING_NAME" comment="" successRequired="false" minCardinality="0" maxCardinality="2147483647">
    <task>taskrepository.tsk#1225112227903</task>
    <state>TestPlans/RobotMovement/RobotMovementTest.pml#1473862711642</state>
  </entryPoints>
</alica:Plan>
