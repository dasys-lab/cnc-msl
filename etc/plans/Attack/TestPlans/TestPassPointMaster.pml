<?xml version="1.0" encoding="ASCII"?>
<alica:Plan xmi:version="2.0" xmlns:xmi="http://www.omg.org/XMI" xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance" xmlns:alica="http:///de.uni_kassel.vs.cn" id="1441106724156" name="TestPassPointMaster" comment="" destinationPath="Plans/Attack/TestPlans" priority="0.0" minCardinality="3" maxCardinality="3" masterPlan="true" utilityFunction="" utilityThreshold="0.1">
  <states id="1441106914297" name="Stop" comment="" entryPoint="1441106724159">
    <plans xsi:type="alica:Behaviour">GenericBehaviours/Stop.beh#1413992604875</plans>
    <inTransitions>Attack/TestPlans/TestPassPointMaster.pml#1441106955404</inTransitions>
    <outTransitions>Attack/TestPlans/TestPassPointMaster.pml#1441106953751</outTransitions>
  </states>
  <states id="1441106916517" name="Start" comment="">
    <plans xsi:type="alica:Plan">Attack/TestPlans/PassPlan.pml#1441106995954</plans>
    <inTransitions>Attack/TestPlans/TestPassPointMaster.pml#1441106953751</inTransitions>
    <outTransitions>Attack/TestPlans/TestPassPointMaster.pml#1441106955404</outTransitions>
  </states>
  <transitions id="1441106953751" name="MISSING_NAME" comment="situation start" msg="">
    <preCondition id="1441106955179" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>Attack/TestPlans/TestPassPointMaster.pml#1441106914297</inState>
    <outState>Attack/TestPlans/TestPassPointMaster.pml#1441106916517</outState>
  </transitions>
  <transitions id="1441106955404" name="MISSING_NAME" comment="situation stop" msg="">
    <preCondition id="1441106956349" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>Attack/TestPlans/TestPassPointMaster.pml#1441106916517</inState>
    <outState>Attack/TestPlans/TestPassPointMaster.pml#1441106914297</outState>
  </transitions>
  <entryPoints id="1441106724159" name="MISSING_NAME" comment="" successRequired="false" minCardinality="3" maxCardinality="3">
    <task>taskrepository.tsk#1225112227903</task>
    <state>Attack/TestPlans/TestPassPointMaster.pml#1441106914297</state>
  </entryPoints>
</alica:Plan>
