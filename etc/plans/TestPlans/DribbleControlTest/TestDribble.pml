<?xml version="1.0" encoding="ASCII"?>
<alica:Plan xmi:version="2.0" xmlns:xmi="http://www.omg.org/XMI" xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance" xmlns:alica="http:///de.uni_kassel.vs.cn" id="1498664114905" name="TestDribble" comment="" masterPlan="false" utilityFunction="" utilityThreshold="0.1" destinationPath="Plans/TestPlans/DribbleControlTest" priority="0.0" minCardinality="0" maxCardinality="2147483647">
  <states id="1498664138705" name="GetBall" comment="" entryPoint="1498664138706">
    <plans xsi:type="alica:BehaviourConfiguration">../../Behaviours/Intercept.beh#1458757193843</plans>
    <inTransitions>#1498664188353</inTransitions>
    <outTransitions>#1498664180789</outTransitions>
  </states>
  <states id="1498664172284" name="Dribble" comment="">
    <plans xsi:type="alica:BehaviourConfiguration">DribbleToAttackPointTest.beh#1498664342592</plans>
    <inTransitions>#1498664180789</inTransitions>
    <outTransitions>#1498664188353</outTransitions>
  </states>
  <transitions id="1498664180789" name="MISSING_NAME" comment="Have Ball" msg="">
    <preCondition id="1498664182584" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>#1498664138705</inState>
    <outState>#1498664172284</outState>
  </transitions>
  <transitions id="1498664188353" name="MISSING_NAME" comment="Lost Ball" msg="">
    <preCondition id="1498664190906" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>#1498664172284</inState>
    <outState>#1498664138705</outState>
  </transitions>
  <entryPoints id="1498664138706" name="MISSING_NAME" comment="" successRequired="false" minCardinality="0" maxCardinality="2147483647">
    <task>../../../Misc/taskrepository.tsk#1225112227903</task>
    <state>#1498664138705</state>
  </entryPoints>
</alica:Plan>
