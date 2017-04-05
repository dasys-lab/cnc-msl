<?xml version="1.0" encoding="ASCII"?>
<alica:Plan xmi:version="2.0" xmlns:xmi="http://www.omg.org/XMI" xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance" xmlns:alica="http:///de.uni_kassel.vs.cn" id="1457015277573" name="OppStandardExecution" comment="" destinationPath="Plans/Standards/Opponent" priority="0.0" minCardinality="0" maxCardinality="2147483647" masterPlan="false" utilityFunction="" utilityThreshold="0.1">
  <states id="1457015277575" name="Position" comment="" entryPoint="1457015277576">
    <plans xsi:type="alica:PlanType">Standards/Opponent/OppStandardPositionType.pty#1457015361187</plans>
    <inTransitions>Standards/Opponent/OppStandardExecution.pml#1458555989921</inTransitions>
    <outTransitions>Standards/Opponent/OppStandardExecution.pml#1457015308829</outTransitions>
    <outTransitions>Standards/Opponent/OppStandardExecution.pml#1458555987772</outTransitions>
  </states>
  <states id="1457015292860" name="WatchBall" comment="">
    <plans xsi:type="alica:PlanType">Standards/Opponent/OppStandardWatchBallType.pty#1457015489608</plans>
    <plans xsi:type="alica:Behaviour">Standards/Opponent/TeamWatchBall.beh#1457015532224</plans>
    <inTransitions>Standards/Opponent/OppStandardExecution.pml#1457015308829</inTransitions>
    <inTransitions>Standards/Opponent/OppStandardExecution.pml#1458555993363</inTransitions>
    <outTransitions>Standards/Opponent/OppStandardExecution.pml#1457015478636</outTransitions>
    <outTransitions>Standards/Opponent/OppStandardExecution.pml#1458555991449</outTransitions>
  </states>
  <states xsi:type="alica:SuccessState" id="1457015318562" name="NewSuccessState" comment="">
    <inTransitions>Standards/Opponent/OppStandardExecution.pml#1457015478636</inTransitions>
  </states>
  <states id="1458555969546" name="Wander" comment="">
    <plans xsi:type="alica:Plan">GameStrategy/Other/WanderPlan.pml#1458553921358</plans>
    <inTransitions>Standards/Opponent/OppStandardExecution.pml#1458555987772</inTransitions>
    <outTransitions>Standards/Opponent/OppStandardExecution.pml#1458555989921</outTransitions>
  </states>
  <states id="1458555971683" name="Wander" comment="">
    <plans xsi:type="alica:Plan">GameStrategy/Other/WanderPlan.pml#1458553921358</plans>
    <inTransitions>Standards/Opponent/OppStandardExecution.pml#1458555991449</inTransitions>
    <outTransitions>Standards/Opponent/OppStandardExecution.pml#1458555993363</outTransitions>
  </states>
  <transitions id="1457015308829" name="MISSING_NAME" comment="Situation==Start" msg="">
    <preCondition id="1457015310255" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>Standards/Opponent/OppStandardExecution.pml#1457015277575</inState>
    <outState>Standards/Opponent/OppStandardExecution.pml#1457015292860</outState>
  </transitions>
  <transitions id="1457015478636" name="MISSING_NAME" comment="WatchBallMessage || 10Secs since start" msg="">
    <preCondition id="1457015479684" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>Standards/Opponent/OppStandardExecution.pml#1457015292860</inState>
    <outState>Standards/Opponent/OppStandardExecution.pml#1457015318562</outState>
  </transitions>
  <transitions id="1458555987772" name="MISSING_NAME" comment="EgoBallPosition==nullptr" msg="">
    <preCondition id="1458555989600" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>Standards/Opponent/OppStandardExecution.pml#1457015277575</inState>
    <outState>Standards/Opponent/OppStandardExecution.pml#1458555969546</outState>
  </transitions>
  <transitions id="1458555989921" name="MISSING_NAME" comment="EgoBallPosition!=nullptr" msg="">
    <preCondition id="1458555991152" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>Standards/Opponent/OppStandardExecution.pml#1458555969546</inState>
    <outState>Standards/Opponent/OppStandardExecution.pml#1457015277575</outState>
  </transitions>
  <transitions id="1458555991449" name="MISSING_NAME" comment="EgoBallPosition==nullptr" msg="">
    <preCondition id="1458555993122" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>Standards/Opponent/OppStandardExecution.pml#1457015292860</inState>
    <outState>Standards/Opponent/OppStandardExecution.pml#1458555971683</outState>
  </transitions>
  <transitions id="1458555993363" name="MISSING_NAME" comment="EgoBallPosition!=nullptr" msg="">
    <preCondition id="1458555994351" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>Standards/Opponent/OppStandardExecution.pml#1458555971683</inState>
    <outState>Standards/Opponent/OppStandardExecution.pml#1457015292860</outState>
  </transitions>
  <entryPoints id="1457015277576" name="MISSING_NAME" comment="" successRequired="true" minCardinality="0" maxCardinality="2147483647">
    <task>taskrepository.tsk#1225112227903</task>
    <state>Standards/Opponent/OppStandardExecution.pml#1457015277575</state>
  </entryPoints>
</alica:Plan>
