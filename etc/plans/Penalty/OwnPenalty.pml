<?xml version="1.0" encoding="ASCII"?>
<alica:Plan xmi:version="2.0" xmlns:xmi="http://www.omg.org/XMI" xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance" xmlns:alica="http:///de.uni_kassel.vs.cn" id="1431525185678" name="OwnPenalty" comment="" destinationPath="Plans/Penalty" priority="0.0" minCardinality="1" maxCardinality="2147483647" masterPlan="false" utilityFunction="" utilityThreshold="0.1">
  <states id="1431525245109" name="DriveToMiddle" comment="" entryPoint="1431525245110">
    <plans xsi:type="alica:Behaviour">Penalty/DriveToPenaltyStart.beh#1459609457478</plans>
    <outTransitions>Penalty/OwnPenalty.pml#1433338525417</outTransitions>
  </states>
  <states id="1431526626723" name="GrabBall" comment="">
    <plans xsi:type="alica:Behaviour">GenericBehaviours/InterceptCarefully.beh#1427703218101</plans>
    <plans xsi:type="alica:Behaviour">Dribble/DribbleControl.beh#1449742071382</plans>
    <plans xsi:type="alica:Behaviour">Behaviours/ShovelSelect.beh#1434199834892</plans>
    <inTransitions>Penalty/OwnPenalty.pml#1433337664670</inTransitions>
    <inTransitions>Penalty/OwnPenalty.pml#1433338528647</inTransitions>
    <outTransitions>Penalty/OwnPenalty.pml#1431526790723</outTransitions>
  </states>
  <states id="1431526769660" name="AlignAndShoot" comment="have ball">
    <plans xsi:type="alica:Behaviour">Penalty/PenaltyAlignAndShoot.beh#1431531496053</plans>
    <plans xsi:type="alica:Behaviour">Behaviours/ShovelSelect.beh#1434199834892</plans>
    <inTransitions>Penalty/OwnPenalty.pml#1431526790723</inTransitions>
    <outTransitions>Penalty/OwnPenalty.pml#1432744248119</outTransitions>
    <outTransitions>Penalty/OwnPenalty.pml#1433337664670</outTransitions>
  </states>
  <states id="1431526917231" name="Stop" comment="" entryPoint="1431526909013">
    <plans xsi:type="alica:Behaviour">GenericBehaviours/Stop.beh#1413992604875</plans>
  </states>
  <states xsi:type="alica:SuccessState" id="1432744233327" name="Success" comment="">
    <inTransitions>Penalty/OwnPenalty.pml#1432744248119</inTransitions>
  </states>
  <states id="1433338496106" name="WaitForStart" comment="">
    <plans xsi:type="alica:Behaviour">GenericBehaviours/Stop.beh#1413992604875</plans>
    <plans xsi:type="alica:Behaviour">Behaviours/ShovelSelect.beh#1434199834892</plans>
    <inTransitions>Penalty/OwnPenalty.pml#1433338525417</inTransitions>
    <outTransitions>Penalty/OwnPenalty.pml#1433338528647</outTransitions>
  </states>
  <transitions id="1431526790723" name="MISSING_NAME" comment="" msg="">
    <preCondition id="1431526792158" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>Penalty/OwnPenalty.pml#1431526626723</inState>
    <outState>Penalty/OwnPenalty.pml#1431526769660</outState>
  </transitions>
  <transitions id="1432744248119" name="MISSING_NAME" comment="anyChildSuccess" msg="">
    <preCondition id="1432744250382" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>Penalty/OwnPenalty.pml#1431526769660</inState>
    <outState>Penalty/OwnPenalty.pml#1432744233327</outState>
  </transitions>
  <transitions id="1433337664670" name="MISSING_NAME" comment="not have ball" msg="">
    <preCondition id="1433337666610" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>Penalty/OwnPenalty.pml#1431526769660</inState>
    <outState>Penalty/OwnPenalty.pml#1431526626723</outState>
  </transitions>
  <transitions id="1433338525417" name="MISSING_NAME" comment="any child success" msg="">
    <preCondition id="1433338526794" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>Penalty/OwnPenalty.pml#1431525245109</inState>
    <outState>Penalty/OwnPenalty.pml#1433338496106</outState>
  </transitions>
  <transitions id="1433338528647" name="MISSING_NAME" comment="start signal received" msg="">
    <preCondition id="1433338530469" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>Penalty/OwnPenalty.pml#1433338496106</inState>
    <outState>Penalty/OwnPenalty.pml#1431526626723</outState>
  </transitions>
  <entryPoints id="1431525245110" name="ExecuteStandard" comment="" successRequired="true" minCardinality="1" maxCardinality="1">
    <task>taskrepository.tsk#1439997010902</task>
    <state>Penalty/OwnPenalty.pml#1431525245109</state>
  </entryPoints>
  <entryPoints id="1431526909013" name="NewEntryPoint" comment="" successRequired="false" minCardinality="0" maxCardinality="2147483647">
    <task>taskrepository.tsk#1225112227903</task>
    <state>Penalty/OwnPenalty.pml#1431526917231</state>
  </entryPoints>
</alica:Plan>
