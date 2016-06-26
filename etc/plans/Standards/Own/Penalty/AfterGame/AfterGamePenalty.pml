<?xml version="1.0" encoding="ASCII"?>
<alica:Plan xmi:version="2.0" xmlns:xmi="http://www.omg.org/XMI" xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance" xmlns:alica="http:///de.uni_kassel.vs.cn" id="1466934340668" name="AfterGamePenalty" comment="" masterPlan="false" utilityFunction="" utilityThreshold="0.1" destinationPath="" priority="0.0" minCardinality="0" maxCardinality="2147483647">
  <states id="1466934340669" name="DriveToMid" comment="" entryPoint="1466934340670">
    <plans xsi:type="alica:BehaviourConfiguration">../PenaltyPosExecuter.beh#1466940432683</plans>
    <outTransitions>#1466934423663</outTransitions>
  </states>
  <states id="1466934355530" name="Wait4Start" comment="">
    <plans xsi:type="alica:BehaviourConfiguration">../../../../Behaviours/ShovelSelect.beh#1435156714286</plans>
    <plans xsi:type="alica:BehaviourConfiguration">../../../../GenericBehaviours/Stop.beh#1413992626194</plans>
    <inTransitions>#1466934423663</inTransitions>
    <outTransitions>#1466934424684</outTransitions>
  </states>
  <states id="1466934357099" name="GrabBall" comment="">
    <plans xsi:type="alica:BehaviourConfiguration">../../../../Behaviours/ShovelSelect.beh#1435156714286</plans>
    <plans xsi:type="alica:BehaviourConfiguration">../../../../GenericBehaviours/InterceptCarefully.beh#1427703234654</plans>
    <plans xsi:type="alica:BehaviourConfiguration">../../../../Dribble/DribbleControl.beh#1450175539163</plans>
    <inTransitions>#1466934424684</inTransitions>
    <inTransitions>#1466934426391</inTransitions>
    <outTransitions>#1466934425319</outTransitions>
  </states>
  <states id="1466934358396" name="AlignAndShoot" comment="">
    <plans xsi:type="alica:BehaviourConfiguration">../../../../Behaviours/ShovelSelect.beh#1435156714286</plans>
    <plans xsi:type="alica:BehaviourConfiguration">../PenaltyShoot.beh#1466940268216</plans>
    <inTransitions>#1466934425319</inTransitions>
    <outTransitions>#1466934426391</outTransitions>
    <outTransitions>#1466934487527</outTransitions>
  </states>
  <states id="1466934359556" name="StayAway" comment="" entryPoint="1466934362334">
    <plans xsi:type="alica:BehaviourConfiguration">StayAwayAfterGamePenalty.beh#1466940600117</plans>
  </states>
  <states xsi:type="alica:SuccessState" id="1466934479510" name="PenaltySuccess" comment="">
    <inTransitions>#1466934487527</inTransitions>
  </states>
  <transitions id="1466934423663" name="MISSING_NAME" comment="" msg="">
    <preCondition id="1466934424435" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>#1466934340669</inState>
    <outState>#1466934355530</outState>
  </transitions>
  <transitions id="1466934424684" name="MISSING_NAME" comment="" msg="">
    <preCondition id="1466934425215" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>#1466934355530</inState>
    <outState>#1466934357099</outState>
  </transitions>
  <transitions id="1466934425319" name="MISSING_NAME" comment="" msg="">
    <preCondition id="1466934426230" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>#1466934357099</inState>
    <outState>#1466934358396</outState>
  </transitions>
  <transitions id="1466934426391" name="MISSING_NAME" comment="" msg="">
    <preCondition id="1466934427236" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>#1466934358396</inState>
    <outState>#1466934357099</outState>
  </transitions>
  <transitions id="1466934487527" name="MISSING_NAME" comment="" msg="">
    <preCondition id="1466934488578" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>#1466934358396</inState>
    <outState>#1466934479510</outState>
  </transitions>
  <entryPoints id="1466934340670" name="ExecuteStandard" comment="" successRequired="false" minCardinality="0" maxCardinality="2147483647">
    <task>../../../../../Misc/taskrepository.tsk#1439997010902</task>
    <state>#1466934340669</state>
  </entryPoints>
  <entryPoints id="1466934362334" name="NewEntryPoint" comment="" successRequired="false" minCardinality="0" maxCardinality="2147483647">
    <task>../../../../../Misc/taskrepository.tsk#1225112227903</task>
    <state>#1466934359556</state>
  </entryPoints>
</alica:Plan>
