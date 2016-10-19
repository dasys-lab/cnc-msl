<?xml version="1.0" encoding="ASCII"?>
<alica:Plan xmi:version="2.0" xmlns:xmi="http://www.omg.org/XMI" xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance" xmlns:alica="http:///de.uni_kassel.vs.cn" id="1464779892293" name="FreekickOwnHalf" comment="" masterPlan="false" utilityFunction="" utilityThreshold="0.1" destinationPath="Plans/Standards/Own/FreeKick" priority="0.0" minCardinality="3" maxCardinality="4">
  <conditions xsi:type="alica:RuntimeCondition" id="1464780785574" name="NewRuntimeCondition" comment="" conditionString="" pluginName="DefaultPlugin">
    <quantifiers xsi:type="alica:ForallAgents" id="1464780802248" name="MISSING_NAME" comment="" scope="1464780761597">
      <sorts>x</sorts>
      <sorts>y</sorts>
    </quantifiers>
    <quantifiers xsi:type="alica:ForallAgents" id="1469533565421" name="MISSING_NAME" comment="" scope="1464781012255">
      <sorts>x</sorts>
      <sorts>y</sorts>
    </quantifiers>
  </conditions>
  <states id="1464779892294" name="AlginExec" comment="" entryPoint="1464779892295">
    <plans xsi:type="alica:BehaviourConfiguration">../../../GenericStandards/StandardActuate.beh#1435766278023</plans>
    <plans xsi:type="alica:BehaviourConfiguration">../../../Behaviours/ShovelSelect.beh#1435156714286</plans>
    <plans xsi:type="alica:BehaviourConfiguration">../../../GenericStandards/StandardAlignToPoint2Receivers.beh#1467229016494</plans>
    <outTransitions>#1464781040324</outTransitions>
  </states>
  <states id="1464780728003" name="AlginRec" comment="" entryPoint="1464780621150">
    <plans xsi:type="alica:BehaviourConfiguration">../ThrowIn/ReceiveInOppHalf.beh#1462370388995</plans>
    <outTransitions>#1464781426314</outTransitions>
  </states>
  <states id="1464780736932" name="AlignAlternativeRec" comment="" entryPoint="1464780622782">
    <plans xsi:type="alica:BehaviourConfiguration">../ThrowIn/PositionAlternativeReceiver.beh#1462978671719</plans>
    <outTransitions>#1464781582341</outTransitions>
  </states>
  <states id="1464780761597" name="Block" comment="" entryPoint="1464780624383">
    <plans xsi:type="alica:BehaviourConfiguration">../../../Behaviours/Pos4Def.beh#1445438204426</plans>
  </states>
  <states id="1464780991973" name="GrabBall" comment="">
    <plans xsi:type="alica:BehaviourConfiguration">../../../Behaviours/ShovelSelect.beh#1435156714286</plans>
    <plans xsi:type="alica:BehaviourConfiguration">../../../GenericStandards/StandardActuate.beh#1435766278023</plans>
    <plans xsi:type="alica:BehaviourConfiguration">../../../GenericStandards/StandardAlignAndGrab2Receivers.beh#1462368748899</plans>
    <inTransitions>#1464781040324</inTransitions>
    <inTransitions>#1464781328367</inTransitions>
    <outTransitions>#1464781042853</outTransitions>
  </states>
  <states id="1464780997998" name="Pass" comment="">
    <plans xsi:type="alica:BehaviourConfiguration">../../../Behaviours/ShovelSelect.beh#1435156714286</plans>
    <plans xsi:type="alica:BehaviourConfiguration">../ThrowIn/ThrowInPass.beh#1462363309950</plans>
    <inTransitions>#1464781042853</inTransitions>
    <outTransitions>#1464781044679</outTransitions>
    <outTransitions>#1464781328367</outTransitions>
  </states>
  <states id="1464781012255" name="Block" comment="">
    <plans xsi:type="alica:BehaviourConfiguration">../../../Behaviours/Pos4Def.beh#1445438204426</plans>
    <inTransitions>#1464781044679</inTransitions>
  </states>
  <states id="1464781391707" name="Receive" comment="">
    <plans xsi:type="alica:BehaviourConfiguration">../../../GenericBehaviours/InterceptCarefully.beh#1427703234654</plans>
    <plans xsi:type="alica:BehaviourConfiguration">../../../Behaviours/ShovelSelect.beh#1435156811453</plans>
    <inTransitions>#1464781426314</inTransitions>
    <outTransitions>#1464781494047</outTransitions>
  </states>
  <states xsi:type="alica:SuccessState" id="1464781475476" name="Success" comment="">
    <inTransitions>#1464781494047</inTransitions>
  </states>
  <states id="1464781530341" name="ReceiveAlternative" comment="">
    <plans xsi:type="alica:BehaviourConfiguration">../../../GenericBehaviours/InterceptCarefully.beh#1427703234654</plans>
    <inTransitions>#1464781582341</inTransitions>
    <outTransitions>#1464781583908</outTransitions>
  </states>
  <states xsi:type="alica:SuccessState" id="1464781564174" name="SuccessAlternative" comment="">
    <inTransitions>#1464781583908</inTransitions>
  </states>
  <transitions id="1464781040324" name="MISSING_NAME" comment="(child success &amp;&amp; start) || (start &amp;&amp; timeSinceStart > timeUntilEmergencyExecute)" msg="">
    <preCondition id="1464781041779" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>#1464779892294</inState>
    <outState>#1464780991973</outState>
  </transitions>
  <transitions id="1464781042853" name="MISSING_NAME" comment="haveball" msg="">
    <preCondition id="1464781044511" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>#1464780991973</inState>
    <outState>#1464780997998</outState>
  </transitions>
  <transitions id="1464781044679" name="MISSING_NAME" comment="executed" msg="">
    <preCondition id="1464781045433" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>#1464780997998</inState>
    <outState>#1464781012255</outState>
  </transitions>
  <transitions id="1464781328367" name="MISSING_NAME" comment="lostball" msg="">
    <preCondition id="1464781329800" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>#1464780997998</inState>
    <outState>#1464780991973</outState>
  </transitions>
  <transitions id="1464781426314" name="MISSING_NAME" comment="pm for own id" msg="">
    <preCondition id="1464781427853" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>#1464780728003</inState>
    <outState>#1464781391707</outState>
  </transitions>
  <transitions id="1464781494047" name="MISSING_NAME" comment="pass received " msg="">
    <preCondition id="1464781495801" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>#1464781391707</inState>
    <outState>#1464781475476</outState>
  </transitions>
  <transitions id="1464781582341" name="MISSING_NAME" comment="pm for own id alternative" msg="">
    <preCondition id="1464781583659" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>#1464780736932</inState>
    <outState>#1464781530341</outState>
  </transitions>
  <transitions id="1464781583908" name="MISSING_NAME" comment="pass received from alternative" msg="">
    <preCondition id="1464781589367" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>#1464781530341</inState>
    <outState>#1464781564174</outState>
  </transitions>
  <entryPoints id="1464779892295" name="ExecuteStandard" comment="" successRequired="false" minCardinality="1" maxCardinality="1">
    <task>../../../../Misc/taskrepository.tsk#1439997010902</task>
    <state>#1464779892294</state>
  </entryPoints>
  <entryPoints id="1464780621150" name="ReceiveStandard" comment="" successRequired="true" minCardinality="1" maxCardinality="1">
    <task>../../../../Misc/taskrepository.tsk#1439997023446</task>
    <state>#1464780728003</state>
  </entryPoints>
  <entryPoints id="1464780622782" name="AlternativeReceive" comment="" successRequired="true" minCardinality="1" maxCardinality="1">
    <task>../../../../Misc/taskrepository.tsk#1462360858945</task>
    <state>#1464780736932</state>
  </entryPoints>
  <entryPoints id="1464780624383" name="Block" comment="" successRequired="false" minCardinality="0" maxCardinality="1">
    <task>../../../../Misc/taskrepository.tsk#1461237765109</task>
    <state>#1464780761597</state>
  </entryPoints>
</alica:Plan>
