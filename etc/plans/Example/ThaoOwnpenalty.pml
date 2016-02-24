<?xml version="1.0" encoding="ASCII"?>
<alica:Plan xmi:version="2.0" xmlns:xmi="http://www.omg.org/XMI" xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance" xmlns:alica="http:///de.uni_kassel.vs.cn" id="1454506732570" name="ThaoOwnpenalty" comment="" masterPlan="false" utilityFunction="" utilityThreshold="0.1" destinationPath="Plans/Example" priority="0.0" minCardinality="0" maxCardinality="2147483647">
  <states id="1454506732571" name="DriveToMiddle" comment="" entryPoint="1454506732572">
    <plans xsi:type="alica:BehaviourConfiguration">../GenericBehaviours/Stop.beh#1413992626194</plans>
    <outTransitions>#1454507415324</outTransitions>
  </states>
  <states id="1454507325367" name="WaitForStart" comment="">
    <plans xsi:type="alica:BehaviourConfiguration">../GenericBehaviours/DriveToPoint.beh#1431527260342</plans>
    <inTransitions>#1454507415324</inTransitions>
    <outTransitions>#1454507416623</outTransitions>
  </states>
  <states id="1454507326785" name="GrabBall" comment="">
    <plans xsi:type="alica:BehaviourConfiguration">../GenericBehaviours/InterceptCarefully.beh#1427703234654</plans>
    <inTransitions>#1454507416623</inTransitions>
    <inTransitions>#1454507420022</inTransitions>
    <outTransitions>#1454507418557</outTransitions>
  </states>
  <states id="1454507336161" name="AlignAndShoot" comment="">
    <plans xsi:type="alica:BehaviourConfiguration">../Penalty/PenaltyAlignAndShoot.beh#1431531542052</plans>
    <inTransitions>#1454507418557</inTransitions>
    <outTransitions>#1454507420022</outTransitions>
  </states>
  <transitions id="1454507415324" name="MISSING_NAME" comment="" msg="">
    <preCondition id="1454507416455" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>#1454506732571</inState>
    <outState>#1454507325367</outState>
  </transitions>
  <transitions id="1454507416623" name="MISSING_NAME" comment="" msg="">
    <preCondition id="1454507418308" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>#1454507325367</inState>
    <outState>#1454507326785</outState>
  </transitions>
  <transitions id="1454507418557" name="MISSING_NAME" comment="" msg="">
    <preCondition id="1454507419613" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>#1454507326785</inState>
    <outState>#1454507336161</outState>
  </transitions>
  <transitions id="1454507420022" name="MISSING_NAME" comment="" msg="">
    <preCondition id="1454507421144" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>#1454507336161</inState>
    <outState>#1454507326785</outState>
  </transitions>
  <entryPoints id="1454506732572" name="MISSING_NAME" comment="" successRequired="false" minCardinality="0" maxCardinality="2147483647">
    <task>../../Misc/taskrepository.tsk#1225112227903</task>
    <state>#1454506732571</state>
  </entryPoints>
</alica:Plan>
