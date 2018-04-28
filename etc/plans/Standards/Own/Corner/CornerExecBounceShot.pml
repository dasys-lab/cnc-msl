<?xml version="1.0" encoding="ASCII"?>
<alica:Plan xmi:version="2.0" xmlns:xmi="http://www.omg.org/XMI" xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance" xmlns:alica="http:///de.uni_kassel.vs.cn" id="1459362028865" name="CornerExecBounceShot" comment="" masterPlan="false" utilityFunction="" utilityThreshold="0.1" destinationPath="Plans/Standards/Own/Corner" priority="0.0" minCardinality="2" maxCardinality="4">
  <states id="1459362049516" name="WaitForReceiver" comment="" entryPoint="1459362049517">
    <plans xsi:type="alica:BehaviourConfiguration">BounceShotAlignPasser.beh#1459357015987</plans>
    <outTransitions>#1459365407849</outTransitions>
  </states>
  <states id="1459365278157" name="RealignWall" comment="" entryPoint="1459365320409">
    <plans xsi:type="alica:BehaviourConfiguration">BounceShotAlignWall.beh#1459356753335</plans>
    <outTransitions>#1459365401604</outTransitions>
  </states>
  <states id="1459365280181" name="Wait" comment="">
    <inTransitions>#1459365411957</inTransitions>
    <outTransitions>#1459365391425</outTransitions>
  </states>
  <states id="1459365281222" name="BouncePassOffRobot" comment="">
    <plans xsi:type="alica:BehaviourConfiguration">BouncePassShoot.beh#1459357188003</plans>
    <inTransitions>#1459365410825</inTransitions>
    <outTransitions>#1459365411957</outTransitions>
  </states>
  <states id="1459365282539" name="FinishAlign" comment="">
    <plans xsi:type="alica:BehaviourConfiguration">BouncePassFinishAlign.beh#1459357089325</plans>
    <inTransitions>#1459365407849</inTransitions>
    <outTransitions>#1459365410825</outTransitions>
  </states>
  <states id="1459365284057" name="WaitForShotToFinish" comment="">
    <inTransitions>#1459365412813</inTransitions>
    <outTransitions>#1459365392554</outTransitions>
  </states>
  <states id="1459365285847" name="WaitForPass" comment="">
    <plans xsi:type="alica:BehaviourConfiguration">../../../GenericBehaviours/Stop.beh#1413992626194</plans>
    <inTransitions>#1459365401604</inTransitions>
    <outTransitions>#1459365412813</outTransitions>
  </states>
  <states id="1459365287615" name="DefenderWall" comment="" entryPoint="1459365322922">
    <plans xsi:type="alica:BehaviourConfiguration">StandardDefendPos.beh#1459356685875</plans>
    <outTransitions>#1459365394266</outTransitions>
  </states>
  <states xsi:type="alica:SuccessState" id="1459365294648" name="NewSuccessState" comment="">
    <inTransitions>#1459365391425</inTransitions>
  </states>
  <states xsi:type="alica:SuccessState" id="1459365296108" name="NewSuccessState" comment="">
    <inTransitions>#1459365392554</inTransitions>
  </states>
  <states xsi:type="alica:SuccessState" id="1459365297570" name="NewSuccessState" comment="">
    <inTransitions>#1459365394266</inTransitions>
  </states>
  <transitions id="1459365391425" name="MISSING_NAME" comment="Wait2Succ" msg="">
    <preCondition id="1459365392367" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>#1459365280181</inState>
    <outState>#1459365294648</outState>
  </transitions>
  <transitions id="1459365392554" name="MISSING_NAME" comment="WaitForShotToFinish2Succ" msg="">
    <preCondition id="1459365394125" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>#1459365284057</inState>
    <outState>#1459365296108</outState>
  </transitions>
  <transitions id="1459365394266" name="MISSING_NAME" comment="DefenderWall2Succ" msg="">
    <preCondition id="1459365401307" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>#1459365287615</inState>
    <outState>#1459365297570</outState>
  </transitions>
  <transitions id="1459365401604" name="MISSING_NAME" comment="RealignWall2WaitforPass" msg="">
    <preCondition id="1459365407637" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>#1459365278157</inState>
    <outState>#1459365285847</outState>
  </transitions>
  <transitions id="1459365407849" name="MISSING_NAME" comment="WaitForReceiver2FinishAlign" msg="">
    <preCondition id="1459365410684" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>#1459362049516</inState>
    <outState>#1459365282539</outState>
  </transitions>
  <transitions id="1459365410825" name="MISSING_NAME" comment="FinishAlign2BouncePassOffRobot" msg="">
    <preCondition id="1459365411722" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>#1459365282539</inState>
    <outState>#1459365281222</outState>
  </transitions>
  <transitions id="1459365411957" name="MISSING_NAME" comment="BouncePassOffRobot2Wait" msg="">
    <preCondition id="1459365412597" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>#1459365281222</inState>
    <outState>#1459365280181</outState>
  </transitions>
  <transitions id="1459365412813" name="MISSING_NAME" comment="WaitForPass2WaitforShotToFinish" msg="">
    <preCondition id="1459365413923" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>#1459365285847</inState>
    <outState>#1459365284057</outState>
  </transitions>
  <entryPoints id="1459362049517" name="ExecuteStandard" comment="" successRequired="false" minCardinality="1" maxCardinality="1">
    <task>../../../../Misc/taskrepository.tsk#1439997010902</task>
    <state>#1459362049516</state>
  </entryPoints>
  <entryPoints id="1459365320409" name="ReceiveStandard" comment="" successRequired="false" minCardinality="1" maxCardinality="1">
    <task>../../../../Misc/taskrepository.tsk#1439997023446</task>
    <state>#1459365278157</state>
  </entryPoints>
  <entryPoints id="1459365322922" name="Defend" comment="" successRequired="false" minCardinality="0" maxCardinality="2">
    <task>../../../../Misc/taskrepository.tsk#1225115406909</task>
    <state>#1459365287615</state>
  </entryPoints>
</alica:Plan>
