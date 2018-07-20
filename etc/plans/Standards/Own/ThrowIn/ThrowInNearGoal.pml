<?xml version="1.0" encoding="ASCII"?>
<alica:Plan xmi:version="2.0" xmlns:xmi="http://www.omg.org/XMI" xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance" xmlns:alica="http:///de.uni_kassel.vs.cn" id="1461237603689" name="ThrowInNearGoal" comment="" masterPlan="false" utilityFunction="" utilityThreshold="0.1" destinationPath="Plans/Standards/Own/ThrowIn" priority="0.0" minCardinality="2" maxCardinality="4">
  <conditions xsi:type="alica:RuntimeCondition" id="1461574228077" name="NewRuntimeCondition" comment="" conditionString="" pluginName="DefaultPlugin">
    <quantifiers xsi:type="alica:ForallAgents" id="1461584608028" name="MISSING_NAME" comment="" scope="1461237746825">
      <sorts>x</sorts>
      <sorts>y</sorts>
    </quantifiers>
    <quantifiers xsi:type="alica:ForallAgents" id="1464194592547" name="MISSING_NAME" comment="" scope="1461237744256">
      <sorts>x</sorts>
      <sorts>y</sorts>
    </quantifiers>
  </conditions>
  <states id="1461237603690" name="Align" comment="" entryPoint="1461237603691">
    <plans xsi:type="alica:BehaviourConfiguration">../../../GenericStandards/StandardAlignToPoint.beh#1435155363994</plans>
    <plans xsi:type="alica:BehaviourConfiguration">../../../Behaviours/ShovelSelect.beh#1435156714286</plans>
    <outTransitions>#1523091130609</outTransitions>
  </states>
  <states xsi:type="alica:SuccessState" id="1461237614163" name="Success" comment="">
    <inTransitions>#1461237853423</inTransitions>
  </states>
  <states id="1461237638988" name="GrabBall" comment="">
    <plans xsi:type="alica:BehaviourConfiguration">../../../Behaviours/ShovelSelect.beh#1435156714286</plans>
    <plans xsi:type="alica:BehaviourConfiguration">../../../GenericStandards/StandardAlignAndGrab.beh#1461583806472</plans>
    <inTransitions>#1461584284347</inTransitions>
    <inTransitions>#1523091130609</inTransitions>
    <outTransitions>#1461237676305</outTransitions>
  </states>
  <states id="1461237666032" name="Pass" comment="">
    <plans xsi:type="alica:BehaviourConfiguration">../../../GenericStandards/StandardPass.beh#1435760175843</plans>
    <plans xsi:type="alica:BehaviourConfiguration">../../../Behaviours/ShovelSelect.beh#1435156714286</plans>
    <inTransitions>#1461237676305</inTransitions>
    <outTransitions>#1461584284347</outTransitions>
    <outTransitions>#1461584440317</outTransitions>
  </states>
  <states id="1461237695900" name="Align" comment="" entryPoint="1461237744256">
    <plans xsi:type="alica:BehaviourConfiguration">PositionReceiverThrownIn.beh#1461584235418</plans>
    <outTransitions>#1461572769511</outTransitions>
  </states>
  <states id="1461237704581" name="Receive" comment="">
    <plans xsi:type="alica:BehaviourConfiguration">../../../Behaviours/ShovelSelect.beh#1435156811453</plans>
    <plans xsi:type="alica:BehaviourConfiguration">../../../Behaviours/Intercept.beh#1521898536366</plans>
    <inTransitions>#1461572769511</inTransitions>
    <outTransitions>#1461237853423</outTransitions>
  </states>
  <states id="1461237711590" name="Block" comment="" entryPoint="1461237746825">
    <plans xsi:type="alica:BehaviourConfiguration">../../../Behaviours/Pos4Def.beh#1445438204426</plans>
  </states>
  <states id="1461237728343" name="PosAlternativePassReceiver" comment="" entryPoint="1461237748826">
    <plans xsi:type="alica:BehaviourConfiguration">PosAlternativePassReceiver.beh#1461674968023</plans>
  </states>
  <states id="1461584409837" name="Wait" comment="">
    <plans xsi:type="alica:BehaviourConfiguration">../../../GenericBehaviours/Stop.beh#1413992626194</plans>
    <inTransitions>#1461584440317</inTransitions>
  </states>
  <transitions id="1461237676305" name="MISSING_NAME" comment="haveBall" msg="">
    <preCondition id="1461237677283" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>#1461237638988</inState>
    <outState>#1461237666032</outState>
  </transitions>
  <transitions id="1461237853423" name="MISSING_NAME" comment="pass received" msg="">
    <preCondition id="1461237854702" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>#1461237704581</inState>
    <outState>#1461237614163</outState>
  </transitions>
  <transitions id="1461572769511" name="MISSING_NAME" comment="aligned &amp;&amp; executor in wait" msg="">
    <preCondition id="1461572770571" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>#1461237695900</inState>
    <outState>#1461237704581</outState>
  </transitions>
  <transitions id="1461584284347" name="MISSING_NAME" comment="not haveBall" msg="">
    <preCondition id="1461584286706" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>#1461237666032</inState>
    <outState>#1461237638988</outState>
  </transitions>
  <transitions id="1461584440317" name="MISSING_NAME" comment="executed" msg="">
    <preCondition id="1461584441559" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>#1461237666032</inState>
    <outState>#1461584409837</outState>
  </transitions>
  <transitions id="1523091130609" name="MISSING_NAME" comment="executor aligned" msg="">
    <preCondition id="1523091132512" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>#1461237603690</inState>
    <outState>#1461237638988</outState>
  </transitions>
  <entryPoints id="1461237603691" name="ExecuteStandard" comment="" successRequired="false" minCardinality="1" maxCardinality="1">
    <task>../../../../Misc/taskrepository.tsk#1439997010902</task>
    <state>#1461237603690</state>
  </entryPoints>
  <entryPoints id="1461237744256" name="ReceiveStandard" comment="" successRequired="true" minCardinality="1" maxCardinality="1">
    <task>../../../../Misc/taskrepository.tsk#1439997023446</task>
    <state>#1461237695900</state>
  </entryPoints>
  <entryPoints id="1461237746825" name="MISSING_NAME" comment="" successRequired="false" minCardinality="0" maxCardinality="1">
    <task>../../../../Misc/taskrepository.tsk#1461237765109</task>
    <state>#1461237711590</state>
  </entryPoints>
  <entryPoints id="1461237748826" name="ReceivePassInGame" comment="" successRequired="false" minCardinality="0" maxCardinality="1">
    <task>../../../../Misc/taskrepository.tsk#1307185798142</task>
    <state>#1461237728343</state>
  </entryPoints>
</alica:Plan>
