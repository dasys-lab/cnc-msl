<?xml version="1.0" encoding="ASCII"?>
<alica:Plan xmi:version="2.0" xmlns:xmi="http://www.omg.org/XMI" xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance" xmlns:alica="http:///de.uni_kassel.vs.cn" id="1466973051873" name="PenaltyMaster" comment="" destinationPath="Plans/Standards/Own/Penalty/AfterGame" priority="0.0" minCardinality="1" maxCardinality="1" masterPlan="true" utilityFunction="" utilityThreshold="0.1">
  <states id="1466973051874" name="Stop" comment="" entryPoint="1466973051875">
    <plans xsi:type="alica:Behaviour">GenericBehaviours/Stop.beh#1413992604875</plans>
    <inTransitions>Standards/Own/Penalty/AfterGame/PenaltyMaster.pml#1466973077670</inTransitions>
    <outTransitions>Standards/Own/Penalty/AfterGame/PenaltyMaster.pml#1466973076640</outTransitions>
  </states>
  <states id="1466973070716" name="Penalty" comment="">
    <plans xsi:type="alica:Plan">Standards/Own/Penalty/AfterGame/AfterGamePenalty.pml#1466934340668</plans>
    <inTransitions>Standards/Own/Penalty/AfterGame/PenaltyMaster.pml#1466973076640</inTransitions>
    <outTransitions>Standards/Own/Penalty/AfterGame/PenaltyMaster.pml#1466973077670</outTransitions>
  </states>
  <transitions id="1466973076640" name="MISSING_NAME" comment="situation = penalty" msg="">
    <preCondition id="1466973077506" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>Standards/Own/Penalty/AfterGame/PenaltyMaster.pml#1466973051874</inState>
    <outState>Standards/Own/Penalty/AfterGame/PenaltyMaster.pml#1466973070716</outState>
  </transitions>
  <transitions id="1466973077670" name="MISSING_NAME" comment="situation = stop" msg="">
    <preCondition id="1466973078342" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>Standards/Own/Penalty/AfterGame/PenaltyMaster.pml#1466973070716</inState>
    <outState>Standards/Own/Penalty/AfterGame/PenaltyMaster.pml#1466973051874</outState>
  </transitions>
  <entryPoints id="1466973051875" name="MISSING_NAME" comment="" successRequired="false" minCardinality="1" maxCardinality="1">
    <task>taskrepository.tsk#1225112227903</task>
    <state>Standards/Own/Penalty/AfterGame/PenaltyMaster.pml#1466973051874</state>
  </entryPoints>
</alica:Plan>
