<?xml version="1.0" encoding="ASCII"?>
<alica:Plan xmi:version="2.0" xmlns:xmi="http://www.omg.org/XMI" xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance" xmlns:alica="http:///de.uni_kassel.vs.cn" id="1479905043165" name="TestDribbleControlMaster" comment="" destinationPath="Plans/TestPlans/DribbleControlTest" priority="0.0" minCardinality="0" maxCardinality="2147483647" masterPlan="true" utilityFunction="" utilityThreshold="0.1">
  <states id="1479905043166" name="Stop" comment="" entryPoint="1479905043167">
    <plans xsi:type="alica:Behaviour">GenericBehaviours/Stop.beh#1413992604875</plans>
    <inTransitions>TestPlans/DribbleControlTest/TestDribbleControlMaster.pml#1479905466840</inTransitions>
    <outTransitions>TestPlans/DribbleControlTest/TestDribbleControlMaster.pml#1479905162449</outTransitions>
  </states>
  <states id="1479905147809" name="DribbleControl" comment="">
    <plans xsi:type="alica:Behaviour">TestPlans/DribbleControlTest/DribbleControlMOS.beh#1479905178049</plans>
    <inTransitions>TestPlans/DribbleControlTest/TestDribbleControlMaster.pml#1479905162449</inTransitions>
    <outTransitions>TestPlans/DribbleControlTest/TestDribbleControlMaster.pml#1479905466840</outTransitions>
  </states>
  <transitions id="1479905162449" name="MISSING_NAME" comment="" msg="">
    <preCondition id="1479905164080" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>TestPlans/DribbleControlTest/TestDribbleControlMaster.pml#1479905043166</inState>
    <outState>TestPlans/DribbleControlTest/TestDribbleControlMaster.pml#1479905147809</outState>
  </transitions>
  <transitions id="1479905466840" name="MISSING_NAME" comment="" msg="">
    <preCondition id="1479905468245" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>TestPlans/DribbleControlTest/TestDribbleControlMaster.pml#1479905147809</inState>
    <outState>TestPlans/DribbleControlTest/TestDribbleControlMaster.pml#1479905043166</outState>
  </transitions>
  <entryPoints id="1479905043167" name="MISSING_NAME" comment="" successRequired="false" minCardinality="0" maxCardinality="2147483647">
    <task>taskrepository.tsk#1225112227903</task>
    <state>TestPlans/DribbleControlTest/TestDribbleControlMaster.pml#1479905043166</state>
  </entryPoints>
</alica:Plan>
