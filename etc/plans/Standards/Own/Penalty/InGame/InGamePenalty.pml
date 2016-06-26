<?xml version="1.0" encoding="ASCII"?>
<alica:Plan xmi:version="2.0" xmlns:xmi="http://www.omg.org/XMI" xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance" xmlns:alica="http:///de.uni_kassel.vs.cn" id="1466936775181" name="InGamePenalty" comment="" masterPlan="false" utilityFunction="" utilityThreshold="0.1" destinationPath="" priority="0.0" minCardinality="0" maxCardinality="2147483647">
  <states id="1466936775182" name="MoveBehindBall" comment="" entryPoint="1466936775183">
    <plans xsi:type="alica:BehaviourConfiguration">../PenaltyPosExecuter.beh#1466940432683</plans>
    <outTransitions>#1466936848368</outTransitions>
  </states>
  <states id="1466936786859" name="Grab Ball" comment="">
    <plans xsi:type="alica:BehaviourConfiguration">../../../../Behaviours/ShovelSelect.beh#1435156714286</plans>
    <plans xsi:type="alica:BehaviourConfiguration">../../../../GenericBehaviours/InterceptCarefully.beh#1427703234654</plans>
    <plans xsi:type="alica:BehaviourConfiguration">../../../../Dribble/DribbleControl.beh#1450175539163</plans>
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
  <states id="1466936792646" name="StayAway" comment="" entryPoint="1466936799335">
    <plans xsi:type="alica:BehaviourConfiguration">StayAwayInGamePenalty.beh#1466940636644</plans>
  </states>
  <states xsi:type="alica:SuccessState" id="1466936854348" name="PenaltySuccess" comment="">
    <inTransitions>#1466936861165</inTransitions>
  </states>
  <transitions id="1466936848368" name="MISSING_NAME" comment="" msg="">
    <preCondition id="1466936849620" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>#1466936775182</inState>
    <outState>#1466936788188</outState>
  </transitions>
  <transitions id="1466936849764" name="MISSING_NAME" comment="" msg="">
    <preCondition id="1466936850439" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>#1466936788188</inState>
    <outState>#1466936786859</outState>
  </transitions>
  <transitions id="1466936850672" name="MISSING_NAME" comment="" msg="">
    <preCondition id="1466936851265" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>#1466936786859</inState>
    <outState>#1466936790118</outState>
  </transitions>
  <transitions id="1466936851417" name="MISSING_NAME" comment="" msg="">
    <preCondition id="1466936852075" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>#1466936790118</inState>
    <outState>#1466936786859</outState>
  </transitions>
  <transitions id="1466936861165" name="MISSING_NAME" comment="" msg="">
    <preCondition id="1466936862234" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>#1466936790118</inState>
    <outState>#1466936854348</outState>
  </transitions>
  <entryPoints id="1466936775183" name="ExecuteStandard" comment="" successRequired="false" minCardinality="0" maxCardinality="2147483647">
    <task>../../../../../Misc/taskrepository.tsk#1439997010902</task>
    <state>#1466936775182</state>
  </entryPoints>
  <entryPoints id="1466936799335" name="NewEntryPoint" comment="" successRequired="false" minCardinality="0" maxCardinality="2147483647">
    <task>../../../../../Misc/taskrepository.tsk#1225112227903</task>
    <state>#1466936792646</state>
  </entryPoints>
</alica:Plan>
