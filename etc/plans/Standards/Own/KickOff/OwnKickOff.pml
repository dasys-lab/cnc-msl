<?xml version="1.0" encoding="ASCII"?>
<alica:Plan xmi:version="2.0" xmlns:xmi="http://www.omg.org/XMI" xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance" xmlns:alica="http:///de.uni_kassel.vs.cn" id="1438785376159" name="OwnKickOff" comment="" masterPlan="false" utilityFunction="" utilityThreshold="0.1" destinationPath="Plans/Standards/Own/KickOff" priority="0.0" minCardinality="2" maxCardinality="4">
  <conditions xsi:type="alica:RuntimeCondition" id="1469455112199" name="NewRuntimeCondition" comment="" conditionString="" pluginName="DefaultPlugin">
    <quantifiers xsi:type="alica:ForallAgents" id="1469455117729" name="MISSING_NAME" comment="" scope="1438785376161">
      <sorts>x</sorts>
      <sorts>y</sorts>
    </quantifiers>
  </conditions>
  <states id="1438785376160" name="PosExec" comment="" entryPoint="1438785376161">
    <plans xsi:type="alica:BehaviourConfiguration">../../../Behaviours/PositionExecutor.beh#1438790487994</plans>
    <plans xsi:type="alica:BehaviourConfiguration">../../../Behaviours/ShovelSelect.beh#1435156714286</plans>
    <outTransitions>#1440675383814</outTransitions>
  </states>
  <states id="1438785501838" name="PosRecv" comment="" entryPoint="1438785392109">
    <plans xsi:type="alica:BehaviourConfiguration">../../../Behaviours/PositionReceiver.beh#1439379352605</plans>
    <outTransitions>#1440772549851</outTransitions>
  </states>
  <states id="1440675376134" name="GrabBall" comment="">
    <plans xsi:type="alica:BehaviourConfiguration">../../../Behaviours/ShovelSelect.beh#1435156714286</plans>
    <plans xsi:type="alica:BehaviourConfiguration">../../../GenericStandards/StandardAlignAndGrab.beh#1459456566595</plans>
    <plans xsi:type="alica:BehaviourConfiguration">../../../GenericStandards/StandardActuate.beh#1435766278023</plans>
    <inTransitions>#1440675383814</inTransitions>
    <inTransitions>#1440772442289</inTransitions>
    <outTransitions>#1440772545631</outTransitions>
  </states>
  <states id="1440772305591" name="Pass" comment="">
    <plans xsi:type="alica:BehaviourConfiguration">../../../Behaviours/ShovelSelect.beh#1435156714286</plans>
    <plans xsi:type="alica:BehaviourConfiguration">../../../GenericStandards/StandardPass.beh#1435760175843</plans>
    <plans xsi:type="alica:BehaviourConfiguration">../../../GenericStandards/StandardActuate.beh#1435766278023</plans>
    <inTransitions>#1440772545631</inTransitions>
    <outTransitions>#1440772442289</outTransitions>
    <outTransitions>#1441811700499</outTransitions>
  </states>
  <states id="1440772454611" name="Receive" comment="">
    <plans xsi:type="alica:BehaviourConfiguration">../../../Behaviours/Intercept.beh#1458757193843</plans>
    <inTransitions>#1440772549851</inTransitions>
    <outTransitions>#1440772873418</outTransitions>
  </states>
  <states xsi:type="alica:SuccessState" id="1440772500050" name="Success" comment="">
    <inTransitions>#1440772873418</inTransitions>
  </states>
  <states id="1441811642575" name="RunFree" comment="">
    <plans xsi:type="alica:BehaviourConfiguration">../../../Behaviours/Pos4Def.beh#1445438204426</plans>
    <inTransitions>#1441811700499</inTransitions>
  </states>
  <states id="1444834163414" name="PosDef" comment="" entryPoint="1444834088607">
    <plans xsi:type="alica:BehaviourConfiguration">../../../Behaviours/Pos2Defenders.beh#1444835591397</plans>
  </states>
  <transitions id="1440675383814" name="PosExec2GrabBall" comment="From PosExec 2 GrabBall State: If we reached the executor position and start signal was sent." msg="">
    <preCondition id="1440675385644" name="PosExec2GrabBall" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>#1438785376160</inState>
    <outState>#1440675376134</outState>
  </transitions>
  <transitions id="1440772442289" name="MISSING_NAME" comment="lost ball" msg="">
    <preCondition id="1440772443709" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>#1440772305591</inState>
    <outState>#1440675376134</outState>
  </transitions>
  <transitions id="1440772545631" name="MISSING_NAME" comment="success" msg="">
    <preCondition id="1440772549250" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>#1440675376134</inState>
    <outState>#1440772305591</outState>
  </transitions>
  <transitions id="1440772549851" name="MISSING_NAME" comment="receiver aligned and executor has shot" msg="">
    <preCondition id="1440772553737" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>#1438785501838</inState>
    <outState>#1440772454611</outState>
  </transitions>
  <transitions id="1440772873418" name="MISSING_NAME" comment="any child success" msg="">
    <preCondition id="1440772876913" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>#1440772454611</inState>
    <outState>#1440772500050</outState>
  </transitions>
  <transitions id="1441811700499" name="MISSING_NAME" comment="executor finished pass" msg="">
    <preCondition id="1441811702176" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>#1440772305591</inState>
    <outState>#1441811642575</outState>
  </transitions>
  <entryPoints id="1438785376161" name="ExecuteStandard" comment="" successRequired="false" minCardinality="1" maxCardinality="1">
    <task>../../../../Misc/taskrepository.tsk#1439997010902</task>
    <state>#1438785376160</state>
  </entryPoints>
  <entryPoints id="1438785392109" name="MISSING_NAME" comment="" successRequired="true" minCardinality="1" maxCardinality="1">
    <task>../../../../Misc/taskrepository.tsk#1439997023446</task>
    <state>#1438785501838</state>
  </entryPoints>
  <entryPoints id="1444834088607" name="Defend" comment="" successRequired="false" minCardinality="0" maxCardinality="2">
    <task>../../../../Misc/taskrepository.tsk#1225115406909</task>
    <state>#1444834163414</state>
  </entryPoints>
</alica:Plan>
