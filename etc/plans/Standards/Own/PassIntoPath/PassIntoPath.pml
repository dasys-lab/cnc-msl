<?xml version="1.0" encoding="ASCII"?>
<alica:Plan xmi:version="2.0" xmlns:xmi="http://www.omg.org/XMI" xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance" xmlns:alica="http:///de.uni_kassel.vs.cn" id="1457530916296" name="PassIntoPath" comment="" destinationPath="Plans/Standards/Own/PassIntoPath" priority="0.0" minCardinality="0" maxCardinality="2147483647" masterPlan="false" utilityFunction="" utilityThreshold="0.1">
  <vars id="1457531196542" name="passGoalX" comment="" Type=""/>
  <vars id="1457531200123" name="passGoalY" comment="" Type=""/>
  <runtimeCondition id="1457531039142" name="NewRuntimeCondition" comment="" conditionString="" pluginName="DefaultPlugin">
    <quantifiers xsi:type="alica:ForallAgents" id="1457531043232" name="MISSING_NAME" comment="" scope="1457531047960">
      <sorts>x</sorts>
      <sorts>y</sorts>
    </quantifiers>
    <vars>Standards/Own/PassIntoPath/PassIntoPath.pml#1457531196542</vars>
    <vars>Standards/Own/PassIntoPath/PassIntoPath.pml#1457531200123</vars>
  </runtimeCondition>
  <states id="1457530916297" name="Align" comment="" entryPoint="1457530916298">
    <parametrisation id="1457532865347" name="MISSING_NAME" comment="">
      <subplan xsi:type="alica:Behaviour">Standards/Own/PassIntoPath/StandardAlignToPassPos.beh#1457532279657</subplan>
      <subvar>Standards/Own/PassIntoPath/StandardAlignToPassPos.beh#1457532821683</subvar>
      <var>Standards/Own/PassIntoPath/PassIntoPath.pml#1457531196542</var>
    </parametrisation>
    <parametrisation id="1457532869164" name="MISSING_NAME" comment="">
      <subplan xsi:type="alica:Behaviour">Standards/Own/PassIntoPath/StandardAlignToPassPos.beh#1457532279657</subplan>
      <subvar>Standards/Own/PassIntoPath/StandardAlignToPassPos.beh#1457532824422</subvar>
      <var>Standards/Own/PassIntoPath/PassIntoPath.pml#1457531200123</var>
    </parametrisation>
    <plans xsi:type="alica:Behaviour">Behaviours/ShovelSelect.beh#1434199834892</plans>
    <plans xsi:type="alica:Behaviour">Standards/Own/PassIntoPath/StandardAlignToPassPos.beh#1457532279657</plans>
    <outTransitions>Standards/Own/PassIntoPath/PassIntoPath.pml#1457531303657</outTransitions>
  </states>
  <states id="1457531111752" name="Align" comment="" entryPoint="1457530953060">
    <parametrisation id="1457532857027" name="MISSING_NAME" comment="">
      <subplan xsi:type="alica:Behaviour">Standards/Own/PassIntoPath/ReceivePassIntoPathGeneric.beh#1457531583460</subplan>
      <subvar>Standards/Own/PassIntoPath/ReceivePassIntoPathGeneric.beh#1457532783704</subvar>
      <var>Standards/Own/PassIntoPath/PassIntoPath.pml#1457531196542</var>
    </parametrisation>
    <parametrisation id="1457532859854" name="MISSING_NAME" comment="">
      <subplan xsi:type="alica:Behaviour">Standards/Own/PassIntoPath/ReceivePassIntoPathGeneric.beh#1457531583460</subplan>
      <subvar>Standards/Own/PassIntoPath/ReceivePassIntoPathGeneric.beh#1457532786957</subvar>
      <var>Standards/Own/PassIntoPath/PassIntoPath.pml#1457531200123</var>
    </parametrisation>
    <plans xsi:type="alica:Behaviour">Standards/Own/PassIntoPath/ReceivePassIntoPathGeneric.beh#1457531583460</plans>
    <outTransitions>Standards/Own/PassIntoPath/PassIntoPath.pml#1457531340791</outTransitions>
  </states>
  <states id="1457531122593" name="Block" comment="" entryPoint="1457531047960">
    <plans xsi:type="alica:Behaviour">Behaviours/Pos4Def.beh#1445438142979</plans>
  </states>
  <states id="1457531267822" name="GrabBall" comment="">
    <parametrisation id="1457532876756" name="MISSING_NAME" comment="">
      <subplan xsi:type="alica:Behaviour">Standards/Own/PassIntoPath/StandardAlignToGeneric.beh#1457531616421</subplan>
      <subvar>Standards/Own/PassIntoPath/StandardAlignToGeneric.beh#1457532831869</subvar>
      <var>Standards/Own/PassIntoPath/PassIntoPath.pml#1457531196542</var>
    </parametrisation>
    <parametrisation id="1457532880852" name="MISSING_NAME" comment="">
      <subplan xsi:type="alica:Behaviour">Standards/Own/PassIntoPath/StandardAlignToGeneric.beh#1457531616421</subplan>
      <subvar>Standards/Own/PassIntoPath/StandardAlignToGeneric.beh#1457532835258</subvar>
      <var>Standards/Own/PassIntoPath/PassIntoPath.pml#1457531200123</var>
    </parametrisation>
    <plans xsi:type="alica:Behaviour">Behaviours/ShovelSelect.beh#1434199834892</plans>
    <plans xsi:type="alica:Behaviour">Standards/Own/PassIntoPath/StandardAlignToGeneric.beh#1457531616421</plans>
    <inTransitions>Standards/Own/PassIntoPath/PassIntoPath.pml#1457531303657</inTransitions>
    <outTransitions>Standards/Own/PassIntoPath/PassIntoPath.pml#1457531307265</outTransitions>
  </states>
  <states id="1457531281152" name="PassKick" comment="">
    <parametrisation id="1457532887948" name="MISSING_NAME" comment="">
      <subplan xsi:type="alica:Behaviour">Standards/Own/PassIntoPath/PassKickIntoPath.beh#1457531678043</subplan>
      <subvar>Standards/Own/PassIntoPath/PassKickIntoPath.beh#1457532847811</subvar>
      <var>Standards/Own/PassIntoPath/PassIntoPath.pml#1457531200123</var>
    </parametrisation>
    <parametrisation id="1457532897533" name="MISSING_NAME" comment="">
      <subplan xsi:type="alica:Behaviour">Standards/Own/PassIntoPath/PassKickIntoPath.beh#1457531678043</subplan>
      <subvar>Standards/Own/PassIntoPath/PassKickIntoPath.beh#1457532845340</subvar>
      <var>Standards/Own/PassIntoPath/PassIntoPath.pml#1457531196542</var>
    </parametrisation>
    <plans xsi:type="alica:Behaviour">GenericBehaviours/Stop.beh#1413992604875</plans>
    <plans xsi:type="alica:Behaviour">Behaviours/ShovelSelect.beh#1434199834892</plans>
    <plans xsi:type="alica:Behaviour">Standards/Own/PassIntoPath/PassKickIntoPath.beh#1457531678043</plans>
    <inTransitions>Standards/Own/PassIntoPath/PassIntoPath.pml#1457531307265</inTransitions>
    <outTransitions>Standards/Own/PassIntoPath/PassIntoPath.pml#1457531315807</outTransitions>
  </states>
  <states id="1457531293259" name="SpatialDefend" comment="">
    <plans xsi:type="alica:Behaviour">GenericBehaviours/Stop.beh#1413992604875</plans>
    <inTransitions>Standards/Own/PassIntoPath/PassIntoPath.pml#1457531315807</inTransitions>
  </states>
  <states id="1457531324804" name="ReceiveAtTarget" comment="">
    <plans xsi:type="alica:Behaviour">GenericBehaviours/Stop.beh#1413992604875</plans>
    <inTransitions>Standards/Own/PassIntoPath/PassIntoPath.pml#1457531340791</inTransitions>
    <outTransitions>Standards/Own/PassIntoPath/PassIntoPath.pml#1457531350730</outTransitions>
  </states>
  <states xsi:type="alica:SuccessState" id="1457531345727" name="NewSuccessState" comment="">
    <inTransitions>Standards/Own/PassIntoPath/PassIntoPath.pml#1457531350730</inTransitions>
  </states>
  <transitions id="1457531303657" name="MISSING_NAME" comment="Situation==Start" msg="">
    <preCondition id="1457531305067" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>Standards/Own/PassIntoPath/PassIntoPath.pml#1457530916297</inState>
    <outState>Standards/Own/PassIntoPath/PassIntoPath.pml#1457531267822</outState>
  </transitions>
  <transitions id="1457531307265" name="MISSING_NAME" comment="Aligned or 9s" msg="">
    <preCondition id="1457531315717" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>Standards/Own/PassIntoPath/PassIntoPath.pml#1457531267822</inState>
    <outState>Standards/Own/PassIntoPath/PassIntoPath.pml#1457531281152</outState>
  </transitions>
  <transitions id="1457531315807" name="MISSING_NAME" comment="Subplan Successful (Kicked)" msg="">
    <preCondition id="1457531317223" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>Standards/Own/PassIntoPath/PassIntoPath.pml#1457531281152</inState>
    <outState>Standards/Own/PassIntoPath/PassIntoPath.pml#1457531293259</outState>
  </transitions>
  <transitions id="1457531340791" name="MISSING_NAME" comment="Passee in SpatialDefend state || PassMsg Received" msg="">
    <preCondition id="1457531342108" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>Standards/Own/PassIntoPath/PassIntoPath.pml#1457531111752</inState>
    <outState>Standards/Own/PassIntoPath/PassIntoPath.pml#1457531324804</outState>
  </transitions>
  <transitions id="1457531350730" name="MISSING_NAME" comment="10s after start || Ball GetBall is near Goal || to far away from origin" msg="">
    <preCondition id="1457531352472" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>Standards/Own/PassIntoPath/PassIntoPath.pml#1457531324804</inState>
    <outState>Standards/Own/PassIntoPath/PassIntoPath.pml#1457531345727</outState>
  </transitions>
  <entryPoints id="1457530916298" name="ExecuteStandard" comment="" successRequired="false" minCardinality="0" maxCardinality="2147483647">
    <task>taskrepository.tsk#1439997010902</task>
    <state>Standards/Own/PassIntoPath/PassIntoPath.pml#1457530916297</state>
  </entryPoints>
  <entryPoints id="1457530953060" name="ReceiveStandard" comment="" successRequired="false" minCardinality="0" maxCardinality="2147483647">
    <task>taskrepository.tsk#1439997023446</task>
    <state>Standards/Own/PassIntoPath/PassIntoPath.pml#1457531111752</state>
  </entryPoints>
  <entryPoints id="1457531047960" name="Block" comment="" successRequired="false" minCardinality="0" maxCardinality="2147483647">
    <task>taskrepository.tsk#1461237765109</task>
    <state>Standards/Own/PassIntoPath/PassIntoPath.pml#1457531122593</state>
  </entryPoints>
</alica:Plan>
