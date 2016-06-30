<?xml version="1.0" encoding="ASCII"?>
<alica:Plan xmi:version="2.0" xmlns:xmi="http://www.omg.org/XMI" xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance" xmlns:alica="http:///de.uni_kassel.vs.cn" id="1462360503828" name="ThrowInOwnHalf" comment="" masterPlan="false" utilityFunction="" utilityThreshold="0.1" destinationPath="Plans/Standards/Own/ThrowIn" priority="0.0" minCardinality="3" maxCardinality="4">
  <conditions xsi:type="alica:RuntimeCondition" id="1462361418213" name="ThrownInOwnHalf - RuntimeCondition" comment="" conditionString="" pluginName="DefaultPlugin">
    <quantifiers xsi:type="alica:ForallAgents" id="1462361435925" name="MISSING_NAME" comment="" scope="1462360612527">
      <sorts>x</sorts>
      <sorts>y</sorts>
    </quantifiers>
  </conditions>
  <states id="1462360503829" name="Align" comment="" entryPoint="1462360503830">
    <plans xsi:type="alica:BehaviourConfiguration">../../../GenericStandards/StandardActuate.beh#1435766278023</plans>
    <plans xsi:type="alica:BehaviourConfiguration">../../../Behaviours/ShovelSelect.beh#1435156714286</plans>
    <plans xsi:type="alica:BehaviourConfiguration">../../../GenericStandards/StandardAlignToPoint2Receivers.beh#1467229016494</plans>
    <outTransitions>#1462360957397</outTransitions>
  </states>
  <states id="1462360912906" name="GrabBall" comment="">
    <plans xsi:type="alica:BehaviourConfiguration">../../../Behaviours/ShovelSelect.beh#1435156714286</plans>
    <plans xsi:type="alica:BehaviourConfiguration">../../../GenericStandards/StandardActuate.beh#1435766278023</plans>
    <plans xsi:type="alica:BehaviourConfiguration">../../../GenericStandards/StandardAlignAndGrab2Receivers.beh#1462368748899</plans>
    <inTransitions>#1462360957397</inTransitions>
    <inTransitions>#1462369226508</inTransitions>
    <outTransitions>#1462360958905</outTransitions>
  </states>
  <states id="1462360919387" name="Pass" comment="">
    <plans xsi:type="alica:BehaviourConfiguration">../../../Behaviours/ShovelSelect.beh#1435156714286</plans>
    <plans xsi:type="alica:BehaviourConfiguration">ThrowInPass.beh#1462363309950</plans>
    <inTransitions>#1462360958905</inTransitions>
    <outTransitions>#1462360960031</outTransitions>
    <outTransitions>#1462369226508</outTransitions>
  </states>
  <states id="1462360928236" name="Wait" comment="">
    <plans xsi:type="alica:BehaviourConfiguration">../../../GenericBehaviours/Stop.beh#1413992626194</plans>
    <inTransitions>#1462360960031</inTransitions>
  </states>
  <states id="1462361351034" name="AlignReceive" comment="" entryPoint="1462360607517">
    <plans xsi:type="alica:BehaviourConfiguration">ReceiveInOppHalf.beh#1462370388995</plans>
    <outTransitions>#1462368266008</outTransitions>
  </states>
  <states id="1462361358155" name="AlignAlternative" comment="" entryPoint="1462360610006">
    <plans xsi:type="alica:BehaviourConfiguration">PositionAlternativeReceiver.beh#1462978671719</plans>
    <outTransitions>#1462368267461</outTransitions>
  </states>
  <states id="1462361373364" name="Block" comment="" entryPoint="1462360612527">
    <plans xsi:type="alica:BehaviourConfiguration">../../../Behaviours/Pos4Def.beh#1445438204426</plans>
  </states>
  <states id="1462363134771" name="ReceiveAlternative" comment="">
    <plans xsi:type="alica:BehaviourConfiguration">../../../GenericBehaviours/InterceptCarefully.beh#1427703234654</plans>
    <inTransitions>#1462368267461</inTransitions>
    <outTransitions>#1462368201420</outTransitions>
  </states>
  <states xsi:type="alica:SuccessState" id="1462363320340" name="Success" comment="">
    <inTransitions>#1462368130262</inTransitions>
  </states>
  <states id="1462368095616" name="Receive" comment="">
    <plans xsi:type="alica:BehaviourConfiguration">../../../GenericBehaviours/InterceptCarefully.beh#1427703234654</plans>
    <plans xsi:type="alica:BehaviourConfiguration">../../../Behaviours/ShovelSelect.beh#1435156811453</plans>
    <inTransitions>#1462368266008</inTransitions>
    <outTransitions>#1462368130262</outTransitions>
  </states>
  <states xsi:type="alica:SuccessState" id="1462368161988" name="Successalternative" comment="">
    <inTransitions>#1462368201420</inTransitions>
  </states>
  <transitions id="1462360957397" name="Align2GrabBall" comment="(child success &amp;&amp; start) || (start &amp;&amp; timeSinceStart > timeUntilEmergencyExecute)" msg="">
    <preCondition id="1462360958757" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>#1462360503829</inState>
    <outState>#1462360912906</outState>
  </transitions>
  <transitions id="1462360958905" name="MISSING_NAME" comment="haveball" msg="">
    <preCondition id="1462360959862" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>#1462360912906</inState>
    <outState>#1462360919387</outState>
  </transitions>
  <transitions id="1462360960031" name="MISSING_NAME" comment="executed" msg="">
    <preCondition id="1462360961688" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>#1462360919387</inState>
    <outState>#1462360928236</outState>
  </transitions>
  <transitions id="1462368130262" name="MISSING_NAME" comment="pass received " msg="">
    <preCondition id="1462368132067" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>#1462368095616</inState>
    <outState>#1462363320340</outState>
  </transitions>
  <transitions id="1462368201420" name="MISSING_NAME" comment="pass received from alternative" msg="">
    <preCondition id="1462368203054" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>#1462363134771</inState>
    <outState>#1462368161988</outState>
  </transitions>
  <transitions id="1462368266008" name="MISSING_NAME" comment="pm for own id" msg="">
    <preCondition id="1462368267324" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>#1462361351034</inState>
    <outState>#1462368095616</outState>
  </transitions>
  <transitions id="1462368267461" name="MISSING_NAME" comment="pm for own id alternative" msg="">
    <preCondition id="1462368269701" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>#1462361358155</inState>
    <outState>#1462363134771</outState>
  </transitions>
  <transitions id="1462369226508" name="MISSING_NAME" comment="lostball" msg="">
    <preCondition id="1462369227764" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>#1462360919387</inState>
    <outState>#1462360912906</outState>
  </transitions>
  <entryPoints id="1462360503830" name="ExecuteStandard" comment="" successRequired="false" minCardinality="1" maxCardinality="1">
    <task>../../../../Misc/taskrepository.tsk#1439997010902</task>
    <state>#1462360503829</state>
  </entryPoints>
  <entryPoints id="1462360607517" name="ReceiveStandard" comment="" successRequired="true" minCardinality="1" maxCardinality="1">
    <task>../../../../Misc/taskrepository.tsk#1439997023446</task>
    <state>#1462361351034</state>
  </entryPoints>
  <entryPoints id="1462360610006" name="MISSING_NAME" comment="" successRequired="true" minCardinality="1" maxCardinality="1">
    <task>../../../../Misc/taskrepository.tsk#1462360858945</task>
    <state>#1462361358155</state>
  </entryPoints>
  <entryPoints id="1462360612527" name="Block" comment="" successRequired="false" minCardinality="0" maxCardinality="1">
    <task>../../../../Misc/taskrepository.tsk#1461237765109</task>
    <state>#1462361373364</state>
  </entryPoints>
</alica:Plan>
