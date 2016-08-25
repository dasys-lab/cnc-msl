<?xml version="1.0" encoding="ASCII"?>
<alica:Plan xmi:version="2.0" xmlns:xmi="http://www.omg.org/XMI" xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance" xmlns:alica="http:///de.uni_kassel.vs.cn" id="1457530916296" name="PassIntoPath" comment="" masterPlan="false" utilityFunction="" utilityThreshold="0.1" destinationPath="Plans/Standards/Own/PassIntoPath" priority="0.0" minCardinality="0" maxCardinality="2147483647">
  <conditions xsi:type="alica:RuntimeCondition" id="1457531039142" name="NewRuntimeCondition" comment="" conditionString="" pluginName="DefaultPlugin">
    <quantifiers xsi:type="alica:ForallAgents" id="1457531043232" name="MISSING_NAME" comment="" scope="1457531047960">
      <sorts>x</sorts>
      <sorts>y</sorts>
    </quantifiers>
    <vars>#1457531196542</vars>
    <vars>#1457531200123</vars>
  </conditions>
  <vars id="1457531196542" name="passGoalX" comment="" Type=""/>
  <vars id="1457531200123" name="passGoalY" comment="" Type=""/>
  <states id="1457530916297" name="Align" comment="" entryPoint="1457530916298">
    <parametrisation id="1457532865347" name="MISSING_NAME" comment="">
      <subplan xsi:type="alica:BehaviourConfiguration">StandardAlignToPassPos.beh#1457532300654</subplan>
      <subvar>StandardAlignToPassPos.beh#1457532821683</subvar>
      <var>#1457531196542</var>
    </parametrisation>
    <parametrisation id="1457532869164" name="MISSING_NAME" comment="">
      <subplan xsi:type="alica:BehaviourConfiguration">StandardAlignToPassPos.beh#1457532300654</subplan>
      <subvar>StandardAlignToPassPos.beh#1457532824422</subvar>
      <var>#1457531200123</var>
    </parametrisation>
    <plans xsi:type="alica:BehaviourConfiguration">../../../Behaviours/ShovelSelect.beh#1435156714286</plans>
    <plans xsi:type="alica:BehaviourConfiguration">StandardAlignToPassPos.beh#1457532300654</plans>
    <outTransitions>#1457531303657</outTransitions>
  </states>
  <states id="1457531111752" name="Align" comment="" entryPoint="1457530953060">
    <parametrisation id="1457532857027" name="MISSING_NAME" comment="">
      <subplan xsi:type="alica:BehaviourConfiguration">ReceivePassIntoPathGeneric.beh#1457531594373</subplan>
      <subvar>ReceivePassIntoPathGeneric.beh#1457532783704</subvar>
      <var>#1457531196542</var>
    </parametrisation>
    <parametrisation id="1457532859854" name="MISSING_NAME" comment="">
      <subplan xsi:type="alica:BehaviourConfiguration">ReceivePassIntoPathGeneric.beh#1457531594373</subplan>
      <subvar>ReceivePassIntoPathGeneric.beh#1457532786957</subvar>
      <var>#1457531200123</var>
    </parametrisation>
    <plans xsi:type="alica:BehaviourConfiguration">ReceivePassIntoPathGeneric.beh#1457531594373</plans>
    <outTransitions>#1457531340791</outTransitions>
  </states>
  <states id="1457531122593" name="Block" comment="" entryPoint="1457531047960">
    <plans xsi:type="alica:BehaviourConfiguration">../../../Behaviours/Pos4Def.beh#1445438204426</plans>
  </states>
  <states id="1457531267822" name="GrabBall" comment="">
    <parametrisation id="1457532876756" name="MISSING_NAME" comment="">
      <subplan xsi:type="alica:BehaviourConfiguration">StandardAlignToGeneric.beh#1457531639350</subplan>
      <subvar>StandardAlignToGeneric.beh#1457532831869</subvar>
      <var>#1457531196542</var>
    </parametrisation>
    <parametrisation id="1457532880852" name="MISSING_NAME" comment="">
      <subplan xsi:type="alica:BehaviourConfiguration">StandardAlignToGeneric.beh#1457531639350</subplan>
      <subvar>StandardAlignToGeneric.beh#1457532835258</subvar>
      <var>#1457531200123</var>
    </parametrisation>
    <plans xsi:type="alica:BehaviourConfiguration">../../../Behaviours/ShovelSelect.beh#1435156714286</plans>
    <plans xsi:type="alica:BehaviourConfiguration">StandardAlignToGeneric.beh#1457531639350</plans>
    <inTransitions>#1457531303657</inTransitions>
    <outTransitions>#1457531307265</outTransitions>
  </states>
  <states id="1457531281152" name="PassKick" comment="">
    <parametrisation id="1457532887948" name="MISSING_NAME" comment="">
      <subplan xsi:type="alica:BehaviourConfiguration">PassKickIntoPath.beh#1457531685581</subplan>
      <subvar>PassKickIntoPath.beh#1457532847811</subvar>
      <var>#1457531200123</var>
    </parametrisation>
    <parametrisation id="1457532897533" name="MISSING_NAME" comment="">
      <subplan xsi:type="alica:BehaviourConfiguration">PassKickIntoPath.beh#1457531685581</subplan>
      <subvar>PassKickIntoPath.beh#1457532845340</subvar>
      <var>#1457531196542</var>
    </parametrisation>
    <plans xsi:type="alica:BehaviourConfiguration">../../../GenericBehaviours/Stop.beh#1413992626194</plans>
    <plans xsi:type="alica:BehaviourConfiguration">../../../Behaviours/ShovelSelect.beh#1435156714286</plans>
    <plans xsi:type="alica:BehaviourConfiguration">PassKickIntoPath.beh#1457531685581</plans>
    <inTransitions>#1457531307265</inTransitions>
    <outTransitions>#1457531315807</outTransitions>
  </states>
  <states id="1457531293259" name="SpatialDefend" comment="">
    <plans xsi:type="alica:BehaviourConfiguration">../../../GenericBehaviours/Stop.beh#1413992626194</plans>
    <inTransitions>#1457531315807</inTransitions>
  </states>
  <states id="1457531324804" name="ReceiveAtTarget" comment="">
    <plans xsi:type="alica:BehaviourConfiguration">../../../GenericBehaviours/Stop.beh#1413992626194</plans>
    <inTransitions>#1457531340791</inTransitions>
    <outTransitions>#1457531350730</outTransitions>
  </states>
  <states xsi:type="alica:SuccessState" id="1457531345727" name="NewSuccessState" comment="">
    <inTransitions>#1457531350730</inTransitions>
  </states>
  <transitions id="1457531303657" name="MISSING_NAME" comment="Situation==Start" msg="">
    <preCondition id="1457531305067" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>#1457530916297</inState>
    <outState>#1457531267822</outState>
  </transitions>
  <transitions id="1457531307265" name="MISSING_NAME" comment="Aligned or 9s" msg="">
    <preCondition id="1457531315717" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>#1457531267822</inState>
    <outState>#1457531281152</outState>
  </transitions>
  <transitions id="1457531315807" name="MISSING_NAME" comment="Subplan Successful (Kicked)" msg="">
    <preCondition id="1457531317223" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>#1457531281152</inState>
    <outState>#1457531293259</outState>
  </transitions>
  <transitions id="1457531340791" name="MISSING_NAME" comment="Passee in SpatialDefend state || PassMsg Received" msg="">
    <preCondition id="1457531342108" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>#1457531111752</inState>
    <outState>#1457531324804</outState>
  </transitions>
  <transitions id="1457531350730" name="MISSING_NAME" comment="10s after start || Ball GetBall is near Goal || to far away from origin" msg="">
    <preCondition id="1457531352472" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>#1457531324804</inState>
    <outState>#1457531345727</outState>
  </transitions>
  <entryPoints id="1457530916298" name="ExecuteStandard" comment="" successRequired="false" minCardinality="0" maxCardinality="2147483647">
    <task>../../../../Misc/taskrepository.tsk#1439997010902</task>
    <state>#1457530916297</state>
  </entryPoints>
  <entryPoints id="1457530953060" name="ReceiveStandard" comment="" successRequired="false" minCardinality="0" maxCardinality="2147483647">
    <task>../../../../Misc/taskrepository.tsk#1439997023446</task>
    <state>#1457531111752</state>
  </entryPoints>
  <entryPoints id="1457531047960" name="Block" comment="" successRequired="false" minCardinality="0" maxCardinality="2147483647">
    <task>../../../../Misc/taskrepository.tsk#1461237765109</task>
    <state>#1457531122593</state>
  </entryPoints>
</alica:Plan>
