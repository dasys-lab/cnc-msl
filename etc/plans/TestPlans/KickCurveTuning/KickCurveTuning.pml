<?xml version="1.0" encoding="ASCII"?>
<alica:Plan xmi:version="2.0" xmlns:xmi="http://www.omg.org/XMI" xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance" xmlns:alica="http:///de.uni_kassel.vs.cn" id="1457698586746" name="KickCurveTuning" comment="" destinationPath="Plans/TestPlans/KickCurveTuning" priority="0.0" minCardinality="0" maxCardinality="2147483647" masterPlan="false" utilityFunction="" utilityThreshold="0.1">
  <states id="1457698586747" name="Stop" comment="" entryPoint="1457698586748">
    <plans xsi:type="alica:Behaviour">GenericBehaviours/Stop.beh#1413992604875</plans>
    <inTransitions>TestPlans/KickCurveTuning/KickCurveTuning.pml#1457698634760</inTransitions>
    <outTransitions>TestPlans/KickCurveTuning/KickCurveTuning.pml#1457698633209</outTransitions>
  </states>
  <states id="1457698608390" name="LaserBallTracking" comment="">
    <plans xsi:type="alica:Behaviour">TestPlans/KickCurveTuning/LaserBallTracking.beh#1457698662032</plans>
    <inTransitions>TestPlans/KickCurveTuning/KickCurveTuning.pml#1457698633209</inTransitions>
    <outTransitions>TestPlans/KickCurveTuning/KickCurveTuning.pml#1457698634760</outTransitions>
  </states>
  <transitions id="1457698633209" name="MISSING_NAME" comment="situation=start" msg="">
    <preCondition id="1457698634601" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>TestPlans/KickCurveTuning/KickCurveTuning.pml#1457698586747</inState>
    <outState>TestPlans/KickCurveTuning/KickCurveTuning.pml#1457698608390</outState>
  </transitions>
  <transitions id="1457698634760" name="MISSING_NAME" comment="situation=stop" msg="">
    <preCondition id="1457698635818" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>TestPlans/KickCurveTuning/KickCurveTuning.pml#1457698608390</inState>
    <outState>TestPlans/KickCurveTuning/KickCurveTuning.pml#1457698586747</outState>
  </transitions>
  <entryPoints id="1457698586748" name="MISSING_NAME" comment="" successRequired="false" minCardinality="0" maxCardinality="2147483647">
    <task>taskrepository.tsk#1225112227903</task>
    <state>TestPlans/KickCurveTuning/KickCurveTuning.pml#1457698586747</state>
  </entryPoints>
</alica:Plan>
