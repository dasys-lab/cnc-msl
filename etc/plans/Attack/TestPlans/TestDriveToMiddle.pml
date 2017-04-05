<?xml version="1.0" encoding="ASCII"?>
<alica:Plan xmi:version="2.0" xmlns:xmi="http://www.omg.org/XMI" xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance" xmlns:alica="http:///de.uni_kassel.vs.cn" id="1457434329037" name="TestDriveToMiddle" comment="" destinationPath="Plans/Attack/TestPlans" priority="0.0" minCardinality="0" maxCardinality="2147483647" masterPlan="true" utilityFunction="" utilityThreshold="0.1">
  <states id="1457434329038" name="Stop" comment="" entryPoint="1457434329039">
    <plans xsi:type="alica:Behaviour">GenericBehaviours/Stop.beh#1413992604875</plans>
    <inTransitions>Attack/TestPlans/TestDriveToMiddle.pml#1457434475595</inTransitions>
    <outTransitions>Attack/TestPlans/TestDriveToMiddle.pml#1457434473874</outTransitions>
  </states>
  <states id="1457434444773" name="DriveToMiddle" comment="">
    <plans xsi:type="alica:Behaviour">GenericBehaviours/DriveToPoint.beh#1417620568675</plans>
    <inTransitions>Attack/TestPlans/TestDriveToMiddle.pml#1457434473874</inTransitions>
    <outTransitions>Attack/TestPlans/TestDriveToMiddle.pml#1457434475595</outTransitions>
  </states>
  <transitions id="1457434473874" name="MISSING_NAME" comment="Start" msg="">
    <preCondition id="1457434475372" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>Attack/TestPlans/TestDriveToMiddle.pml#1457434329038</inState>
    <outState>Attack/TestPlans/TestDriveToMiddle.pml#1457434444773</outState>
  </transitions>
  <transitions id="1457434475595" name="MISSING_NAME" comment="Stop" msg="">
    <preCondition id="1457434476549" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>Attack/TestPlans/TestDriveToMiddle.pml#1457434444773</inState>
    <outState>Attack/TestPlans/TestDriveToMiddle.pml#1457434329038</outState>
  </transitions>
  <entryPoints id="1457434329039" name="MISSING_NAME" comment="" successRequired="false" minCardinality="0" maxCardinality="2147483647">
    <task>taskrepository.tsk#1225112227903</task>
    <state>Attack/TestPlans/TestDriveToMiddle.pml#1457434329038</state>
  </entryPoints>
</alica:Plan>
