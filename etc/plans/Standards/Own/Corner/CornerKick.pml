<?xml version="1.0" encoding="ASCII"?>
<alica:Plan xmi:version="2.0" xmlns:xmi="http://www.omg.org/XMI" xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance" xmlns:alica="http:///de.uni_kassel.vs.cn" id="1462373376006" name="CornerKick" comment="" masterPlan="false" utilityFunction="" utilityThreshold="0.1" destinationPath="Plans/Standards/Own/Corner" priority="0.0" minCardinality="2" maxCardinality="4">
  <conditions xsi:type="alica:RuntimeCondition" id="1464793807994" name="NewRuntimeCondition" comment="" conditionString="" pluginName="DefaultPlugin">
    <quantifiers xsi:type="alica:ForallAgents" id="1464793819347" name="MISSING_NAME" comment="" scope="1464793576624">
      <sorts>x</sorts>
      <sorts>y</sorts>
    </quantifiers>
    <quantifiers xsi:type="alica:ForallAgents" id="1464795471742" name="MISSING_NAME" comment="" scope="1464793641082">
      <sorts>x</sorts>
      <sorts>y</sorts>
    </quantifiers>
  </conditions>
  <states id="1462373376007" name="AlignExec" comment="" entryPoint="1462373376008">
    <plans xsi:type="alica:BehaviourConfiguration">../../../GenericStandards/StandardActuate.beh#1435766278023</plans>
    <plans xsi:type="alica:BehaviourConfiguration">../../../Behaviours/ShovelSelect.beh#1435156714286</plans>
    <plans xsi:type="alica:BehaviourConfiguration">../../../GenericStandards/StandardAlignToPoint.beh#1435155363994</plans>
    <outTransitions>#1464784050959</outTransitions>
  </states>
  <states id="1462374515826" name="AlignRecv" comment="" entryPoint="1462373457908">
    <plans xsi:type="alica:BehaviourConfiguration">../../../Behaviours/ShovelSelect.beh#1435156811453</plans>
    <plans xsi:type="alica:BehaviourConfiguration">Pos4ReceiverCornerKick.beh#1464787549220</plans>
    <outTransitions>#1464792274211</outTransitions>
  </states>
  <states id="1464783052747" name="GrabBall" comment="">
    <plans xsi:type="alica:BehaviourConfiguration">../../../GenericStandards/StandardActuate.beh#1435766278023</plans>
    <plans xsi:type="alica:BehaviourConfiguration">../../../Behaviours/ShovelSelect.beh#1435156714286</plans>
    <plans xsi:type="alica:BehaviourConfiguration">../../../GenericStandards/StandardAlignAndGrab.beh#1465038982091</plans>
    <inTransitions>#1464784050959</inTransitions>
    <inTransitions>#1464787998856</inTransitions>
    <outTransitions>#1464787987943</outTransitions>
  </states>
  <states id="1464787959998" name="Pass" comment="">
    <plans xsi:type="alica:BehaviourConfiguration">../../../Behaviours/ShovelSelect.beh#1435156714286</plans>
    <plans xsi:type="alica:BehaviourConfiguration">../../../GenericStandards/StandardPass.beh#1435760175843</plans>
    <inTransitions>#1464787987943</inTransitions>
    <outTransitions>#1464787998856</outTransitions>
    <outTransitions>#1464788058735</outTransitions>
  </states>
  <states id="1464788034326" name="Wait" comment="">
    <plans xsi:type="alica:BehaviourConfiguration">../../../GenericBehaviours/Stop.beh#1413992626194</plans>
    <inTransitions>#1464788058735</inTransitions>
  </states>
  <states id="1464792231235" name="Recieve" comment="">
    <plans xsi:type="alica:BehaviourConfiguration">../../../Behaviours/ShovelSelect.beh#1435156811453</plans>
    <plans xsi:type="alica:BehaviourConfiguration">../../../Behaviours/Intercept.beh#1458757193843</plans>
    <inTransitions>#1464792274211</inTransitions>
    <outTransitions>#1464796134632</outTransitions>
  </states>
  <states xsi:type="alica:SuccessState" id="1464792294365" name="Success" comment="">
    <inTransitions>#1464796134632</inTransitions>
  </states>
  <states id="1464793576624" name="Block" comment="" entryPoint="1464793555239">
    <plans xsi:type="alica:BehaviourConfiguration">../../../Behaviours/Pos4Def.beh#1445438204426</plans>
  </states>
  <states id="1464793641082" name="Defend" comment="" entryPoint="1464793619930">
    <plans xsi:type="alica:BehaviourConfiguration">../../../Behaviours/Pos4Def.beh#1445438204426</plans>
  </states>
  <transitions id="1464784050959" name="MISSING_NAME" comment="(child success &amp;&amp; start) || (start &amp;&amp; timeSinceStart > timeUntilEmergencyExecute)" msg="">
    <preCondition id="1464784053079" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>#1462373376007</inState>
    <outState>#1464783052747</outState>
  </transitions>
  <transitions id="1464787987943" name="MISSING_NAME" comment="has ball" msg="">
    <preCondition id="1464787990119" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>#1464783052747</inState>
    <outState>#1464787959998</outState>
  </transitions>
  <transitions id="1464787998856" name="MISSING_NAME" comment="lost Ball" msg="">
    <preCondition id="1464788000901" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>#1464787959998</inState>
    <outState>#1464783052747</outState>
  </transitions>
  <transitions id="1464788058735" name="MISSING_NAME" comment="executed" msg="">
    <preCondition id="1464788060652" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>#1464787959998</inState>
    <outState>#1464788034326</outState>
  </transitions>
  <transitions id="1464792274211" name="MISSING_NAME" comment="pm for own id" msg="">
    <preCondition id="1464792277148" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>#1462374515826</inState>
    <outState>#1464792231235</outState>
  </transitions>
  <transitions id="1464796134632" name="MISSING_NAME" comment="success" msg="">
    <preCondition id="1464796139312" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>#1464792231235</inState>
    <outState>#1464792294365</outState>
  </transitions>
  <entryPoints id="1462373376008" name="ExecuteStandard" comment="" successRequired="false" minCardinality="1" maxCardinality="1">
    <task>../../../../Misc/taskrepository.tsk#1439997010902</task>
    <state>#1462373376007</state>
  </entryPoints>
  <entryPoints id="1462373457908" name="ReceiveStandard" comment="" successRequired="true" minCardinality="1" maxCardinality="1">
    <task>../../../../Misc/taskrepository.tsk#1439997023446</task>
    <state>#1462374515826</state>
  </entryPoints>
  <entryPoints id="1464793555239" name="Block" comment="" successRequired="false" minCardinality="0" maxCardinality="1">
    <task>../../../../Misc/taskrepository.tsk#1461237765109</task>
    <state>#1464793576624</state>
  </entryPoints>
  <entryPoints id="1464793619930" name="Defend" comment="" successRequired="false" minCardinality="0" maxCardinality="1">
    <task>../../../../Misc/taskrepository.tsk#1225115406909</task>
    <state>#1464793641082</state>
  </entryPoints>
</alica:Plan>
