<?xml version="1.0" encoding="ASCII"?>
<alica:Plan xmi:version="2.0" xmlns:xmi="http://www.omg.org/XMI" xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance" xmlns:alica="http:///de.uni_kassel.vs.cn" id="1466934340668" name="AfterGamePenalty" comment="" destinationPath="Plans/Standards/Own/Penalty/AfterGame" priority="0.0" minCardinality="0" maxCardinality="2" masterPlan="false" utilityFunction="" utilityThreshold="0.1">
  <states id="1466934340669" name="DriveToMid" comment="" entryPoint="1466934340670">
    <plans xsi:type="alica:Behaviour">GenericBehaviours/DriveToPoint.beh#1417620568675</plans>
    <outTransitions>Standards/Own/Penalty/AfterGame/AfterGamePenalty.pml#1466934423663</outTransitions>
  </states>
  <states id="1466934355530" name="Wait4Start" comment="">
    <plans xsi:type="alica:Behaviour">Behaviours/ShovelSelect.beh#1434199834892</plans>
    <plans xsi:type="alica:Behaviour">GenericBehaviours/Stop.beh#1413992604875</plans>
    <inTransitions>Standards/Own/Penalty/AfterGame/AfterGamePenalty.pml#1466934423663</inTransitions>
    <outTransitions>Standards/Own/Penalty/AfterGame/AfterGamePenalty.pml#1466934424684</outTransitions>
  </states>
  <states id="1466934357099" name="GrabBall" comment="">
    <plans xsi:type="alica:Behaviour">Behaviours/ShovelSelect.beh#1434199834892</plans>
    <plans xsi:type="alica:Behaviour">GenericBehaviours/InterceptCarefully.beh#1427703218101</plans>
    <plans xsi:type="alica:Behaviour">Dribble/DribbleControl.beh#1449742071382</plans>
    <inTransitions>Standards/Own/Penalty/AfterGame/AfterGamePenalty.pml#1466934424684</inTransitions>
    <inTransitions>Standards/Own/Penalty/AfterGame/AfterGamePenalty.pml#1466934426391</inTransitions>
    <outTransitions>Standards/Own/Penalty/AfterGame/AfterGamePenalty.pml#1466934425319</outTransitions>
  </states>
  <states id="1466934358396" name="AlignAndShoot" comment="">
    <plans xsi:type="alica:Behaviour">Behaviours/ShovelSelect.beh#1434199834892</plans>
    <plans xsi:type="alica:Behaviour">Standards/Own/Penalty/PenaltyShoot.beh#1466940246275</plans>
    <inTransitions>Standards/Own/Penalty/AfterGame/AfterGamePenalty.pml#1466934425319</inTransitions>
    <outTransitions>Standards/Own/Penalty/AfterGame/AfterGamePenalty.pml#1466934426391</outTransitions>
    <outTransitions>Standards/Own/Penalty/AfterGame/AfterGamePenalty.pml#1466934487527</outTransitions>
  </states>
  <states id="1466934359556" name="KeepGoal" comment="" entryPoint="1466934362334">
    <plans xsi:type="alica:Plan">Goalie/Test/GoalieDefault.pml#1447254438614</plans>
    <outTransitions>Standards/Own/Penalty/AfterGame/AfterGamePenalty.pml#1466973019163</outTransitions>
  </states>
  <states xsi:type="alica:SuccessState" id="1466934479510" name="ShootSuccess" comment="">
    <inTransitions>Standards/Own/Penalty/AfterGame/AfterGamePenalty.pml#1466934487527</inTransitions>
  </states>
  <states xsi:type="alica:SuccessState" id="1466973000329" name="KeepSuccess" comment="">
    <inTransitions>Standards/Own/Penalty/AfterGame/AfterGamePenalty.pml#1466973019163</inTransitions>
  </states>
  <transitions id="1466934423663" name="MISSING_NAME" comment="" msg="">
    <preCondition id="1466934424435" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>Standards/Own/Penalty/AfterGame/AfterGamePenalty.pml#1466934340669</inState>
    <outState>Standards/Own/Penalty/AfterGame/AfterGamePenalty.pml#1466934355530</outState>
  </transitions>
  <transitions id="1466934424684" name="MISSING_NAME" comment="" msg="">
    <preCondition id="1466934425215" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>Standards/Own/Penalty/AfterGame/AfterGamePenalty.pml#1466934355530</inState>
    <outState>Standards/Own/Penalty/AfterGame/AfterGamePenalty.pml#1466934357099</outState>
  </transitions>
  <transitions id="1466934425319" name="MISSING_NAME" comment="" msg="">
    <preCondition id="1466934426230" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>Standards/Own/Penalty/AfterGame/AfterGamePenalty.pml#1466934357099</inState>
    <outState>Standards/Own/Penalty/AfterGame/AfterGamePenalty.pml#1466934358396</outState>
  </transitions>
  <transitions id="1466934426391" name="MISSING_NAME" comment="" msg="">
    <preCondition id="1466934427236" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>Standards/Own/Penalty/AfterGame/AfterGamePenalty.pml#1466934358396</inState>
    <outState>Standards/Own/Penalty/AfterGame/AfterGamePenalty.pml#1466934357099</outState>
  </transitions>
  <transitions id="1466934487527" name="MISSING_NAME" comment="" msg="">
    <preCondition id="1466934488578" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>Standards/Own/Penalty/AfterGame/AfterGamePenalty.pml#1466934358396</inState>
    <outState>Standards/Own/Penalty/AfterGame/AfterGamePenalty.pml#1466934479510</outState>
  </transitions>
  <transitions id="1466973019163" name="MISSING_NAME" comment="" msg="">
    <preCondition id="1466973020291" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>Standards/Own/Penalty/AfterGame/AfterGamePenalty.pml#1466934359556</inState>
    <outState>Standards/Own/Penalty/AfterGame/AfterGamePenalty.pml#1466973000329</outState>
  </transitions>
  <entryPoints id="1466934340670" name="ExecuteStandard" comment="" successRequired="true" minCardinality="0" maxCardinality="1">
    <task>taskrepository.tsk#1439997010902</task>
    <state>Standards/Own/Penalty/AfterGame/AfterGamePenalty.pml#1466934340669</state>
  </entryPoints>
  <entryPoints id="1466934362334" name="Keeper" comment="" successRequired="false" minCardinality="0" maxCardinality="1">
    <task>taskrepository.tsk#1221754402444</task>
    <state>Standards/Own/Penalty/AfterGame/AfterGamePenalty.pml#1466934359556</state>
  </entryPoints>
</alica:Plan>
