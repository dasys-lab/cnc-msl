<?xml version="1.0" encoding="ASCII"?>
<alica:Plan xmi:version="2.0" xmlns:xmi="http://www.omg.org/XMI" xmlns:alica="http:///de.uni_kassel.vs.cn" id="1464531946023" name="OwnFreeKickInOppHalf" comment="" masterPlan="false" utilityFunction="" utilityThreshold="0.1" destinationPath="Plans/Standards/Own/FreeKick" priority="0.0" minCardinality="2" maxCardinality="4">
  <states id="1464535161175" name="PositionExecutor" comment="" entryPoint="1464531946025">
    <outTransitions>#1464778510115</outTransitions>
  </states>
  <states id="1464535169536" name="PositionReceiver" comment="" entryPoint="1464532126334">
    <outTransitions>#1464778513652</outTransitions>
  </states>
  <states id="1464535201681" name="GrabBall" comment="">
    <inTransitions>#1464778510115</inTransitions>
    <outTransitions>#1464778511430</outTransitions>
  </states>
  <states id="1464535219397" name="Pass" comment="">
    <inTransitions>#1464778511430</inTransitions>
  </states>
  <states id="1464535253598" name="Receive" comment="">
    <inTransitions>#1464778513652</inTransitions>
    <outTransitions>#1464778515443</outTransitions>
  </states>
  <states id="1464535263395" name="Shoot" comment="">
    <inTransitions>#1464778515443</inTransitions>
  </states>
  <states id="1464535682818" name="PositionInsideOppPenalty" comment="" entryPoint="1464532128302"/>
  <states id="1464535706293" name="PositionCloseToOppPenalty" comment="" entryPoint="1464532130252"/>
  <transitions id="1464778510115" name="MISSING_NAME" comment="" msg="">
    <preCondition id="1464778511333" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>#1464535161175</inState>
    <outState>#1464535201681</outState>
  </transitions>
  <transitions id="1464778511430" name="MISSING_NAME" comment="" msg="">
    <preCondition id="1464778513499" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>#1464535201681</inState>
    <outState>#1464535219397</outState>
  </transitions>
  <transitions id="1464778513652" name="MISSING_NAME" comment="" msg="">
    <preCondition id="1464778515331" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>#1464535169536</inState>
    <outState>#1464535253598</outState>
  </transitions>
  <transitions id="1464778515443" name="MISSING_NAME" comment="" msg="">
    <preCondition id="1464778516153" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>#1464535253598</inState>
    <outState>#1464535263395</outState>
  </transitions>
  <entryPoints id="1464531946025" name="ExecuteStandard" comment="" successRequired="false" minCardinality="1" maxCardinality="1">
    <task>../../../../Misc/taskrepository.tsk#1439997010902</task>
    <state>#1464535161175</state>
  </entryPoints>
  <entryPoints id="1464532126334" name="ReceiveStandard" comment="" successRequired="false" minCardinality="1" maxCardinality="1">
    <task>../../../../Misc/taskrepository.tsk#1439997023446</task>
    <state>#1464535169536</state>
  </entryPoints>
  <entryPoints id="1464532128302" name="StandInsideOppPenalty" comment="" successRequired="false" minCardinality="0" maxCardinality="1">
    <task>../../../../Misc/taskrepository.tsk#1464532449309</task>
    <state>#1464535682818</state>
  </entryPoints>
  <entryPoints id="1464532130252" name="StandOutsideOppPenalty" comment="" successRequired="false" minCardinality="0" maxCardinality="1">
    <task>../../../../Misc/taskrepository.tsk#1225112227903</task>
    <state>#1464535706293</state>
  </entryPoints>
</alica:Plan>
