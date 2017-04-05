<?xml version="1.0" encoding="ASCII"?>
<alica:Plan xmi:version="2.0" xmlns:xmi="http://www.omg.org/XMI" xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance" xmlns:alica="http:///de.uni_kassel.vs.cn" id="1454506180437" name="DuelTestMaster" comment="" destinationPath="Plans/Attack/TestPlans" priority="0.0" minCardinality="0" maxCardinality="2147483647" masterPlan="false" utilityFunction="" utilityThreshold="0.1">
  <states id="1454506180438" name="Stop" comment="" entryPoint="1454506180439">
    <plans xsi:type="alica:Behaviour">GenericBehaviours/Stop.beh#1413992604875</plans>
    <inTransitions>Attack/TestPlans/DuelTestMaster.pml#1458125318498</inTransitions>
    <inTransitions>Attack/TestPlans/DuelTestMaster.pml#1458135841024</inTransitions>
    <outTransitions>Attack/TestPlans/DuelTestMaster.pml#1454506223310</outTransitions>
  </states>
  <states id="1454506213756" name="Duel" comment="">
    <plans xsi:type="alica:Plan">Attack/Duel.pml#1450178655416</plans>
    <inTransitions>Attack/TestPlans/DuelTestMaster.pml#1454506223310</inTransitions>
    <outTransitions>Attack/TestPlans/DuelTestMaster.pml#1458125292658</outTransitions>
    <outTransitions>Attack/TestPlans/DuelTestMaster.pml#1458135841024</outTransitions>
  </states>
  <states id="1458125263117" name="Wait" comment="">
    <plans xsi:type="alica:Behaviour">GenericBehaviours/Stop.beh#1413992604875</plans>
    <inTransitions>Attack/TestPlans/DuelTestMaster.pml#1458125292658</inTransitions>
    <outTransitions>Attack/TestPlans/DuelTestMaster.pml#1458125318498</outTransitions>
  </states>
  <transitions id="1454506223310" name="MISSING_NAME" comment="situation == start" msg="">
    <preCondition id="1454506224420" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>Attack/TestPlans/DuelTestMaster.pml#1454506180438</inState>
    <outState>Attack/TestPlans/DuelTestMaster.pml#1454506213756</outState>
  </transitions>
  <transitions id="1458125292658" name="MISSING_NAME" comment="any child success " msg="">
    <preCondition id="1458125293553" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>Attack/TestPlans/DuelTestMaster.pml#1454506213756</inState>
    <outState>Attack/TestPlans/DuelTestMaster.pml#1458125263117</outState>
  </transitions>
  <transitions id="1458125318498" name="MISSING_NAME" comment="situation stop" msg="">
    <preCondition id="1458125320118" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>Attack/TestPlans/DuelTestMaster.pml#1458125263117</inState>
    <outState>Attack/TestPlans/DuelTestMaster.pml#1454506180438</outState>
  </transitions>
  <transitions id="1458135841024" name="MISSING_NAME" comment="situation stop" msg="">
    <preCondition id="1458135843050" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>Attack/TestPlans/DuelTestMaster.pml#1454506213756</inState>
    <outState>Attack/TestPlans/DuelTestMaster.pml#1454506180438</outState>
  </transitions>
  <entryPoints id="1454506180439" name="MISSING_NAME" comment="" successRequired="true" minCardinality="0" maxCardinality="2147483647">
    <task>taskrepository.tsk#1225112227903</task>
    <state>Attack/TestPlans/DuelTestMaster.pml#1454506180438</state>
  </entryPoints>
</alica:Plan>
