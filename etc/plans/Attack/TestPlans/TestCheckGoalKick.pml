<?xml version="1.0" encoding="ASCII"?>
<alica:Plan xmi:version="2.0" xmlns:xmi="http://www.omg.org/XMI" xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance" xmlns:alica="http:///de.uni_kassel.vs.cn" id="1449076138236" name="TestCheckGoalKick" comment="" destinationPath="Plans/Attack/TestPlans" priority="0.0" minCardinality="0" maxCardinality="2147483647" masterPlan="true" utilityFunction="" utilityThreshold="0.1">
  <states id="1449076138237" name="Stop" comment="" entryPoint="1449076138238">
    <plans xsi:type="alica:Behaviour">GenericBehaviours/Stop.beh#1413992604875</plans>
    <plans xsi:type="alica:Behaviour">Behaviours/ShovelSelect.beh#1434199834892</plans>
    <inTransitions>Attack/TestPlans/TestCheckGoalKick.pml#1449076203977</inTransitions>
    <outTransitions>Attack/TestPlans/TestCheckGoalKick.pml#1449076202114</outTransitions>
  </states>
  <states id="1449076173801" name="CheckGoalKick" comment="">
    <plans xsi:type="alica:Behaviour">GenericBehaviours/CheckGoalKick.beh#1449076008755</plans>
    <plans xsi:type="alica:Behaviour">Behaviours/Joystick.beh#1421854975890</plans>
    <plans xsi:type="alica:Behaviour">Behaviours/ShovelSelect.beh#1434199834892</plans>
    <inTransitions>Attack/TestPlans/TestCheckGoalKick.pml#1449076202114</inTransitions>
    <outTransitions>Attack/TestPlans/TestCheckGoalKick.pml#1449076203977</outTransitions>
  </states>
  <transitions id="1449076202114" name="MISSING_NAME" comment="Start" msg="">
    <preCondition id="1449076203800" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>Attack/TestPlans/TestCheckGoalKick.pml#1449076138237</inState>
    <outState>Attack/TestPlans/TestCheckGoalKick.pml#1449076173801</outState>
  </transitions>
  <transitions id="1449076203977" name="MISSING_NAME" comment="Stop" msg="">
    <preCondition id="1449076205925" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>Attack/TestPlans/TestCheckGoalKick.pml#1449076173801</inState>
    <outState>Attack/TestPlans/TestCheckGoalKick.pml#1449076138237</outState>
  </transitions>
  <entryPoints id="1449076138238" name="MISSING_NAME" comment="" successRequired="false" minCardinality="0" maxCardinality="2147483647">
    <task>taskrepository.tsk#1225112227903</task>
    <state>Attack/TestPlans/TestCheckGoalKick.pml#1449076138237</state>
  </entryPoints>
</alica:Plan>
