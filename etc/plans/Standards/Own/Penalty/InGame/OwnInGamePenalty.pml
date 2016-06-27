<?xml version="1.0" encoding="ASCII"?>
<alica:Plan xmi:version="2.0" xmlns:xmi="http://www.omg.org/XMI" xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance" xmlns:alica="http:///de.uni_kassel.vs.cn" id="1466936775181" name="OwnInGamePenalty" comment="" masterPlan="false" utilityFunction="" utilityThreshold="0.1" destinationPath="Plans/Standards/Own/Penalty/InGame" priority="0.0" minCardinality="1" maxCardinality="4">
  <states id="1466936775182" name="MoveBehindBall" comment="" entryPoint="1466936775183">
    <plans xsi:type="alica:BehaviourConfiguration">../PenaltyPosExecuter.beh#1466940432683</plans>
    <outTransitions>#1466936848368</outTransitions>
  </states>
  <states id="1466936786859" name="Grab Ball" comment="">
    <plans xsi:type="alica:BehaviourConfiguration">../../../../Behaviours/ShovelSelect.beh#1435156714286</plans>
    <plans xsi:type="alica:BehaviourConfiguration">../../../../GenericBehaviours/InterceptCarefully.beh#1427703234654</plans>
    <plans xsi:type="alica:BehaviourConfiguration">../../../../Dribble/DribbleControl.beh#1450175539163</plans>
    <plans xsi:type="alica:BehaviourConfiguration">../../../../GenericStandards/StandardActuate.beh#1435766278023</plans>
    <inTransitions>#1466936849764</inTransitions>
    <inTransitions>#1466936851417</inTransitions>
    <outTransitions>#1466936850672</outTransitions>
  </states>
  <states id="1466936788188" name="Wait4Start" comment="">
    <plans xsi:type="alica:BehaviourConfiguration">../../../../GenericBehaviours/Stop.beh#1413992626194</plans>
    <plans xsi:type="alica:BehaviourConfiguration">../../../../Behaviours/ShovelSelect.beh#1435156714286</plans>
    <inTransitions>#1466936848368</inTransitions>
    <outTransitions>#1466936849764</outTransitions>
  </states>
  <states id="1466936790118" name="AlignAndShoot" comment="">
    <plans xsi:type="alica:BehaviourConfiguration">../../../../Behaviours/ShovelSelect.beh#1435156714286</plans>
    <plans xsi:type="alica:BehaviourConfiguration">../PenaltyShoot.beh#1466940268216</plans>
    <inTransitions>#1466936850672</inTransitions>
    <outTransitions>#1466936851417</outTransitions>
    <outTransitions>#1466936861165</outTransitions>
  </states>
  <states id="1466936792646" name="Pos4Rebounce" comment="" entryPoint="1466936799335">
    <plans xsi:type="alica:BehaviourConfiguration">Pos4PenaltyRebounce.beh#1466976004315</plans>
  </states>
  <states xsi:type="alica:SuccessState" id="1466936854348" name="PenaltySuccess" comment="">
    <inTransitions>#1466936861165</inTransitions>
  </states>
  <states id="1466972478368" name="Defend" comment="" entryPoint="1466972404622">
    <plans xsi:type="alica:BehaviourConfiguration">../../../../Behaviours/BackroomDefence.beh#1454507819086</plans>
  </states>
  <transitions id="1466936848368" name="MISSING_NAME" comment="any child success" msg="">
    <preCondition id="1466936849620" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>#1466936775182</inState>
    <outState>#1466936788188</outState>
  </transitions>
  <transitions id="1466936849764" name="MISSING_NAME" comment="situation = start" msg="">
    <preCondition id="1466936850439" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>#1466936788188</inState>
    <outState>#1466936786859</outState>
  </transitions>
  <transitions id="1466936850672" name="MISSING_NAME" comment="have ball" msg="">
    <preCondition id="1466936851265" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>#1466936786859</inState>
    <outState>#1466936790118</outState>
  </transitions>
  <transitions id="1466936851417" name="MISSING_NAME" comment="!have ball" msg="">
    <preCondition id="1466936852075" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>#1466936790118</inState>
    <outState>#1466936786859</outState>
  </transitions>
  <transitions id="1466936861165" name="MISSING_NAME" comment="any child success" msg="">
    <preCondition id="1466936862234" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>#1466936790118</inState>
    <outState>#1466936854348</outState>
  </transitions>
  <entryPoints id="1466936775183" name="ExecuteStandard" comment="" successRequired="true" minCardinality="1" maxCardinality="1">
    <task>../../../../../Misc/taskrepository.tsk#1439997010902</task>
    <state>#1466936775182</state>
  </entryPoints>
  <entryPoints id="1466936799335" name="AttackSupport" comment="" successRequired="false" minCardinality="0" maxCardinality="1">
    <task>../../../../../Misc/taskrepository.tsk#1225115536468</task>
    <state>#1466936792646</state>
  </entryPoints>
  <entryPoints id="1466972404622" name="Defend" comment="" successRequired="false" minCardinality="0" maxCardinality="2">
    <task>../../../../../Misc/taskrepository.tsk#1225115406909</task>
    <state>#1466972478368</state>
  </entryPoints>
</alica:Plan>
