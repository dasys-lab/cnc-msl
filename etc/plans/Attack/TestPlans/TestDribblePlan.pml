<?xml version="1.0" encoding="ASCII"?>
<alica:Plan xmi:version="2.0" xmlns:xmi="http://www.omg.org/XMI" xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance" xmlns:alica="http:///de.uni_kassel.vs.cn" id="1437902404050" name="TestDribblePlan" comment="" destinationPath="Plans/Attack/TestPlans" priority="0.0" minCardinality="0" maxCardinality="2147483647" masterPlan="true" utilityFunction="" utilityThreshold="0.1">
  <states id="1437902404051" name="Stop" comment="" entryPoint="1437902404053">
    <plans xsi:type="alica:Behaviour">GenericBehaviours/Stop.beh#1413992604875</plans>
    <inTransitions>Attack/TestPlans/TestDribblePlan.pml#1437902448588</inTransitions>
    <outTransitions>Attack/TestPlans/TestDribblePlan.pml#1437902447098</outTransitions>
  </states>
  <states id="1437902416095" name="Start" comment="">
    <plans xsi:type="alica:Plan">Attack/TestPlans/DribbleToPointPlan.pml#1436960829485</plans>
    <inTransitions>Attack/TestPlans/TestDribblePlan.pml#1437902447098</inTransitions>
    <outTransitions>Attack/TestPlans/TestDribblePlan.pml#1437902448588</outTransitions>
  </states>
  <transitions id="1437902447098" name="MISSING_NAME" comment="situation == start" msg="">
    <preCondition id="1437902448394" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>Attack/TestPlans/TestDribblePlan.pml#1437902404051</inState>
    <outState>Attack/TestPlans/TestDribblePlan.pml#1437902416095</outState>
  </transitions>
  <transitions id="1437902448588" name="MISSING_NAME" comment="situation == stop" msg="">
    <preCondition id="1437902449568" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>Attack/TestPlans/TestDribblePlan.pml#1437902416095</inState>
    <outState>Attack/TestPlans/TestDribblePlan.pml#1437902404051</outState>
  </transitions>
  <entryPoints id="1437902404053" name="MISSING_NAME" comment="" successRequired="false" minCardinality="0" maxCardinality="2147483647">
    <task>taskrepository.tsk#1225112227903</task>
    <state>Attack/TestPlans/TestDribblePlan.pml#1437902404051</state>
  </entryPoints>
</alica:Plan>
