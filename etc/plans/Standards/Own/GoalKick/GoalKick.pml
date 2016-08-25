<?xml version="1.0" encoding="ASCII"?>
<alica:Plan xmi:version="2.0" xmlns:xmi="http://www.omg.org/XMI" xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance" xmlns:alica="http:///de.uni_kassel.vs.cn" id="1469521732930" name="GoalKick" comment="" masterPlan="false" utilityFunction="" utilityThreshold="0.1" destinationPath="Plans/Standards/Own/GoalKick" priority="0.0" minCardinality="0" maxCardinality="2147483647">
  <conditions xsi:type="alica:RuntimeCondition" id="1469522753378" name="NewRuntimeCondition" comment="" conditionString="" pluginName="DefaultPlugin">
    <quantifiers xsi:type="alica:ForallAgents" id="1469522849862" name="MISSING_NAME" comment="" scope="1469522047745">
      <sorts>x</sorts>
      <sorts>y</sorts>
    </quantifiers>
  </conditions>
  <states id="1469521732931" name="AlignExec" comment="" entryPoint="1469521732932">
    <plans xsi:type="alica:BehaviourConfiguration">../../../GenericStandards/StandardActuate.beh#1435766278023</plans>
    <plans xsi:type="alica:BehaviourConfiguration">../../../GenericStandards/StandardAlignAndGrab2Receivers.beh#1462368748899</plans>
    <plans xsi:type="alica:BehaviourConfiguration">../../../Behaviours/ShovelSelect.beh#1435156714286</plans>
    <outTransitions>#1469522458021</outTransitions>
  </states>
  <states id="1469522259560" name="Block" comment="" entryPoint="1469522047745">
    <plans xsi:type="alica:BehaviourConfiguration">../../../Behaviours/Pos4Def.beh#1445438204426</plans>
  </states>
  <states id="1469522367640" name="GrabBall" comment="">
    <plans xsi:type="alica:BehaviourConfiguration">../../../Behaviours/ShovelSelect.beh#1435156714286</plans>
    <plans xsi:type="alica:BehaviourConfiguration">../../../GenericStandards/StandardActuate.beh#1435766278023</plans>
    <plans xsi:type="alica:BehaviourConfiguration">../../../GenericStandards/StandardAlignAndGrab2Receivers.beh#1462368748899</plans>
    <inTransitions>#1469522458021</inTransitions>
    <inTransitions>#1469522461468</inTransitions>
    <outTransitions>#1469522459411</outTransitions>
  </states>
  <states id="1469522370067" name="Pass" comment="">
    <plans xsi:type="alica:BehaviourConfiguration">../../../Behaviours/ShovelSelect.beh#1435156714286</plans>
    <inTransitions>#1469522459411</inTransitions>
    <outTransitions>#1469522460547</outTransitions>
    <outTransitions>#1469522461468</outTransitions>
  </states>
  <states id="1469522371971" name="Stop" comment="">
    <plans xsi:type="alica:BehaviourConfiguration">../../../GenericBehaviours/Stop.beh#1413992626194</plans>
    <inTransitions>#1469522460547</inTransitions>
  </states>
  <states id="1469522490708" name="AlignRec" comment="" entryPoint="1469522042827">
    <plans xsi:type="alica:BehaviourConfiguration">../../../Behaviours/ShovelSelect.beh#1435156811453</plans>
    <plans xsi:type="alica:BehaviourConfiguration">../ThrowIn/ReceiveInOppHalf.beh#1462370388995</plans>
    <outTransitions>#1469522683853</outTransitions>
  </states>
  <states id="1469522492956" name="Receive" comment="">
    <plans xsi:type="alica:BehaviourConfiguration">../../../GenericBehaviours/InterceptCarefully.beh#1427703234654</plans>
    <plans xsi:type="alica:BehaviourConfiguration">../../../Behaviours/ShovelSelect.beh#1435156811453</plans>
    <inTransitions>#1469522683853</inTransitions>
    <outTransitions>#1469522685570</outTransitions>
  </states>
  <states id="1469522494669" name="AlignRecAlternative" comment="" entryPoint="1469522045842">
    <plans xsi:type="alica:BehaviourConfiguration">../ThrowIn/PositionAlternativeReceiver.beh#1462978671719</plans>
    <outTransitions>#1469522690355</outTransitions>
  </states>
  <states id="1469522496824" name="ReceiveAlternative" comment="">
    <plans xsi:type="alica:BehaviourConfiguration">../../../GenericBehaviours/InterceptCarefully.beh#1427703234654</plans>
    <inTransitions>#1469522690355</inTransitions>
    <outTransitions>#1469522688609</outTransitions>
  </states>
  <states xsi:type="alica:SuccessState" id="1469522586732" name="Success" comment="">
    <inTransitions>#1469522685570</inTransitions>
  </states>
  <states xsi:type="alica:SuccessState" id="1469522588381" name="SuccessAlternative" comment="">
    <inTransitions>#1469522688609</inTransitions>
  </states>
  <transitions id="1469522458021" name="MISSING_NAME" comment="(child success &amp;&amp; start) || (start &amp;&amp; timeSinceStart > timeUntilEmergencyExecute)" msg="">
    <preCondition id="1469522459250" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>#1469521732931</inState>
    <outState>#1469522367640</outState>
  </transitions>
  <transitions id="1469522459411" name="MISSING_NAME" comment="haveball" msg="">
    <preCondition id="1469522460345" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>#1469522367640</inState>
    <outState>#1469522370067</outState>
  </transitions>
  <transitions id="1469522460547" name="MISSING_NAME" comment="executed" msg="">
    <preCondition id="1469522461332" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>#1469522370067</inState>
    <outState>#1469522371971</outState>
  </transitions>
  <transitions id="1469522461468" name="MISSING_NAME" comment="lostball" msg="">
    <preCondition id="1469522463041" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>#1469522370067</inState>
    <outState>#1469522367640</outState>
  </transitions>
  <transitions id="1469522683853" name="MISSING_NAME" comment="pm for own id" msg="">
    <preCondition id="1469522685256" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>#1469522490708</inState>
    <outState>#1469522492956</outState>
  </transitions>
  <transitions id="1469522685570" name="MISSING_NAME" comment="pass received " msg="">
    <preCondition id="1469522686688" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>#1469522492956</inState>
    <outState>#1469522586732</outState>
  </transitions>
  <transitions id="1469522688609" name="MISSING_NAME" comment="pass received from alternative" msg="">
    <preCondition id="1469522690037" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>#1469522496824</inState>
    <outState>#1469522588381</outState>
  </transitions>
  <transitions id="1469522690355" name="MISSING_NAME" comment="pm for own id alternative" msg="">
    <preCondition id="1469522692577" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>#1469522494669</inState>
    <outState>#1469522496824</outState>
  </transitions>
  <entryPoints id="1469521732932" name="ExecuteStandard" comment="" successRequired="false" minCardinality="0" maxCardinality="2147483647">
    <task>../../../../Misc/taskrepository.tsk#1439997010902</task>
    <state>#1469521732931</state>
  </entryPoints>
  <entryPoints id="1469522042827" name="ReceiveStandard" comment="" successRequired="true" minCardinality="0" maxCardinality="2147483647">
    <task>../../../../Misc/taskrepository.tsk#1439997023446</task>
    <state>#1469522490708</state>
  </entryPoints>
  <entryPoints id="1469522045842" name="AlternativeReceive" comment="" successRequired="true" minCardinality="0" maxCardinality="2147483647">
    <task>../../../../Misc/taskrepository.tsk#1462360858945</task>
    <state>#1469522494669</state>
  </entryPoints>
  <entryPoints id="1469522047745" name="Block" comment="" successRequired="false" minCardinality="0" maxCardinality="2147483647">
    <task>../../../../Misc/taskrepository.tsk#1461237765109</task>
    <state>#1469522259560</state>
  </entryPoints>
</alica:Plan>
