<?xml version="1.0" encoding="ASCII"?>
<alica:Plan xmi:version="2.0" xmlns:xmi="http://www.omg.org/XMI" xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance" xmlns:alica="http:///de.uni_kassel.vs.cn" id="1431525185678" name="OwnPenalty" comment="" masterPlan="false" utilityFunction="" utilityThreshold="0.1" destinationPath="Plans/Penalty" priority="0.0" minCardinality="1" maxCardinality="2147483647">
  <states id="1431525245109" name="DriveToMiddle" comment="" entryPoint="1431525245110">
    <plans xsi:type="alica:BehaviourConfiguration">../GenericBehaviours/DriveToPoint.beh#1431527260342</plans>
    <outTransitions>#1431526662401</outTransitions>
  </states>
  <states id="1431526626723" name="GrabBall" comment="">
    <plans xsi:type="alica:BehaviourConfiguration">../GenericBehaviours/InterceptCarefully.beh#1427703234654</plans>
    <inTransitions>#1431526662401</inTransitions>
    <outTransitions>#1431526790723</outTransitions>
  </states>
  <states id="1431526769660" name="AlignAndShoot" comment="have ball">
    <plans xsi:type="alica:BehaviourConfiguration">PenaltyAlignAndShoot.beh#1431531542052</plans>
    <inTransitions>#1431526790723</inTransitions>
    <outTransitions>#1432744248119</outTransitions>
  </states>
  <states id="1431526917231" name="Stop" comment="" entryPoint="1431526909013">
    <plans xsi:type="alica:BehaviourConfiguration">../GenericBehaviours/Stop.beh#1413992626194</plans>
  </states>
  <states xsi:type="alica:SuccessState" id="1432744233327" name="Success" comment="">
    <inTransitions>#1432744248119</inTransitions>
  </states>
  <transitions id="1431526662401" name="MISSING_NAME" comment="start signal received" msg="">
    <preCondition id="1431526664270" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>#1431525245109</inState>
    <outState>#1431526626723</outState>
  </transitions>
  <transitions id="1431526790723" name="MISSING_NAME" comment="" msg="">
    <preCondition id="1431526792158" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>#1431526626723</inState>
    <outState>#1431526769660</outState>
  </transitions>
  <transitions id="1432744248119" name="MISSING_NAME" comment="anyChildSuccess" msg="">
    <preCondition id="1432744250382" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>#1431526769660</inState>
    <outState>#1432744233327</outState>
  </transitions>
  <entryPoints id="1431525245110" name="StandardExecuter" comment="" successRequired="true" minCardinality="1" maxCardinality="1">
    <task>../../Misc/taskrepository.tsk#1238601692867</task>
    <state>#1431525245109</state>
  </entryPoints>
  <entryPoints id="1431526909013" name="NewEntryPoint" comment="" successRequired="false" minCardinality="0" maxCardinality="2147483647">
    <task>../../Misc/taskrepository.tsk#1225112227903</task>
    <state>#1431526917231</state>
  </entryPoints>
</alica:Plan>
