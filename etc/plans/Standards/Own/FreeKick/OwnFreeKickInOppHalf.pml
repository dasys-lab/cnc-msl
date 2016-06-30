<?xml version="1.0" encoding="ASCII"?>
<alica:Plan xmi:version="2.0" xmlns:xmi="http://www.omg.org/XMI" xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance" xmlns:alica="http:///de.uni_kassel.vs.cn" id="1464531946023" name="OwnFreeKickInOppHalf" comment="" masterPlan="false" utilityFunction="" utilityThreshold="0.1" destinationPath="Plans/Standards/Own/FreeKick" priority="0.0" minCardinality="2" maxCardinality="4">
  <conditions xsi:type="alica:RuntimeCondition" id="1467206311808" name="NewRuntimeCondition" comment="" conditionString="" pluginName="DefaultPlugin">
    <quantifiers xsi:type="alica:ForallAgents" id="1467206315623" name="MISSING_NAME" comment="" scope="1464535682818">
      <sorts>x</sorts>
      <sorts>y</sorts>
    </quantifiers>
    <quantifiers xsi:type="alica:ForallAgents" id="1467206324388" name="MISSING_NAME" comment="" scope="1464535706293">
      <sorts>x</sorts>
      <sorts>y</sorts>
    </quantifiers>
  </conditions>
  <states id="1464535161175" name="PositionExecutor" comment="" entryPoint="1464531946025">
    <plans xsi:type="alica:BehaviourConfiguration">../../../GenericStandards/StandardAlignToPoint.beh#1435155363994</plans>
    <plans xsi:type="alica:BehaviourConfiguration">../../../Behaviours/ShovelSelect.beh#1435156714286</plans>
    <plans xsi:type="alica:BehaviourConfiguration">../../../GenericStandards/StandardActuate.beh#1435766278023</plans>
    <outTransitions>#1464778510115</outTransitions>
  </states>
  <states id="1464535169536" name="PositionReceiver" comment="" entryPoint="1464532126334">
    <plans xsi:type="alica:BehaviourConfiguration">PositionReceiverFreeKickOppHalf.beh#1464780824372</plans>
    <plans xsi:type="alica:BehaviourConfiguration">../../../Behaviours/ShovelSelect.beh#1435156811453</plans>
    <outTransitions>#1464778513652</outTransitions>
  </states>
  <states id="1464535201681" name="GrabBall" comment="">
    <plans xsi:type="alica:BehaviourConfiguration">../../../Behaviours/ShovelSelect.beh#1435156714286</plans>
    <plans xsi:type="alica:BehaviourConfiguration">../../../GenericStandards/StandardActuate.beh#1435766278023</plans>
    <plans xsi:type="alica:BehaviourConfiguration">../../../GenericStandards/StandardAlignAndGrab.beh#1466861369486</plans>
    <inTransitions>#1464778510115</inTransitions>
    <inTransitions>#1464783504464</inTransitions>
    <outTransitions>#1464778511430</outTransitions>
  </states>
  <states id="1464535219397" name="Pass" comment="">
    <plans xsi:type="alica:BehaviourConfiguration">../../../GenericStandards/StandardPass.beh#1435760175843</plans>
    <inTransitions>#1464778511430</inTransitions>
    <outTransitions>#1464783504464</outTransitions>
    <outTransitions>#1464785250936</outTransitions>
  </states>
  <states id="1464535253598" name="Receive" comment="">
    <plans xsi:type="alica:BehaviourConfiguration">../../../GenericStandards/StandardActuate.beh#1435766278023</plans>
    <plans xsi:type="alica:BehaviourConfiguration">../../../Behaviours/ShovelSelect.beh#1435156811453</plans>
    <plans xsi:type="alica:BehaviourConfiguration">../../../GenericBehaviours/InterceptCarefully.beh#1427703234654</plans>
    <inTransitions>#1464778513652</inTransitions>
    <outTransitions>#1464778515443</outTransitions>
  </states>
  <states id="1464535263395" name="Shoot" comment="">
    <plans xsi:type="alica:BehaviourConfiguration">../../../Behaviours/ShovelSelect.beh#1435156811453</plans>
    <plans xsi:type="alica:BehaviourConfiguration">AlignFreeGoalSpace.beh#1467039882734</plans>
    <plans xsi:type="alica:BehaviourConfiguration">../../../GenericBehaviours/CheckGoalKick.beh#1467265292648</plans>
    <inTransitions>#1464778515443</inTransitions>
    <outTransitions>#1464785278735</outTransitions>
  </states>
  <states id="1464535682818" name="PositionInsideOppPenalty" comment="" entryPoint="1464532128302">
    <plans xsi:type="alica:BehaviourConfiguration">Pos2Penalty.beh#1465474190742</plans>
  </states>
  <states id="1464535706293" name="PositionCloseToOppPenalty" comment="" entryPoint="1464532130252">
    <plans xsi:type="alica:BehaviourConfiguration">Pos2Penalty.beh#1465474190742</plans>
  </states>
  <states xsi:type="alica:SuccessState" id="1464785222776" name="Success" comment="">
    <inTransitions>#1464785278735</inTransitions>
  </states>
  <states id="1464785237103" name="Wait/MoveOutOfWay" comment="">
    <inTransitions>#1464785250936</inTransitions>
  </states>
  <transitions id="1464778510115" name="MISSING_NAME" comment="(child success &amp;&amp; start) || (start &amp;&amp; timeSinceStart > emergencyExecute)" msg="">
    <preCondition id="1464778511333" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>#1464535161175</inState>
    <outState>#1464535201681</outState>
  </transitions>
  <transitions id="1464778511430" name="MISSING_NAME" comment="haveBall" msg="">
    <preCondition id="1464778513499" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>#1464535201681</inState>
    <outState>#1464535219397</outState>
  </transitions>
  <transitions id="1464778513652" name="MISSING_NAME" comment="aligned &amp;&amp; executor in wait" msg="">
    <preCondition id="1464778515331" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>#1464535169536</inState>
    <outState>#1464535253598</outState>
  </transitions>
  <transitions id="1464778515443" name="MISSING_NAME" comment="received pass" msg="">
    <preCondition id="1464778516153" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>#1464535253598</inState>
    <outState>#1464535263395</outState>
  </transitions>
  <transitions id="1464783504464" name="MISSING_NAME" comment="lostBall" msg="">
    <preCondition id="1464783506322" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>#1464535219397</inState>
    <outState>#1464535201681</outState>
  </transitions>
  <transitions id="1464785250936" name="MISSING_NAME" comment="pass success" msg="">
    <preCondition id="1464785252648" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>#1464535219397</inState>
    <outState>#1464785237103</outState>
  </transitions>
  <transitions id="1464785278735" name="MISSING_NAME" comment="shot successfully" msg="">
    <preCondition id="1464785280406" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>#1464535263395</inState>
    <outState>#1464785222776</outState>
  </transitions>
  <entryPoints id="1464531946025" name="ExecuteStandard" comment="" successRequired="false" minCardinality="1" maxCardinality="1">
    <task>../../../../Misc/taskrepository.tsk#1439997010902</task>
    <state>#1464535161175</state>
  </entryPoints>
  <entryPoints id="1464532126334" name="ReceiveStandard" comment="" successRequired="true" minCardinality="1" maxCardinality="1">
    <task>../../../../Misc/taskrepository.tsk#1439997023446</task>
    <state>#1464535169536</state>
  </entryPoints>
  <entryPoints id="1464532128302" name="StandInsideOppPenalty" comment="" successRequired="false" minCardinality="0" maxCardinality="1">
    <task>../../../../Misc/taskrepository.tsk#1464532449309</task>
    <state>#1464535682818</state>
  </entryPoints>
  <entryPoints id="1464532130252" name="MISSING_NAME" comment="" successRequired="false" minCardinality="0" maxCardinality="1">
    <task>../../../../Misc/taskrepository.tsk#1464864263733</task>
    <state>#1464535706293</state>
  </entryPoints>
</alica:Plan>
