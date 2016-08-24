<?xml version="1.0" encoding="ASCII"?>
<alica:Plan xmi:version="2.0" xmlns:xmi="http://www.omg.org/XMI" xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance" xmlns:alica="http:///de.uni_kassel.vs.cn" id="1472044587219" name="DriveInSquare" comment="" masterPlan="false" utilityFunction="" utilityThreshold="0.1" destinationPath="Plans/TestPlans/DriveInSquare" priority="0.0" minCardinality="0" maxCardinality="2147483647">
  <states id="1472044615068" name="DriveUp" comment="" entryPoint="1472044615069">
    <plans xsi:type="alica:BehaviourConfiguration">DriveTo.beh#1472045139882</plans>
    <inTransitions>#1472044676789</inTransitions>
    <outTransitions>#1472044672299</outTransitions>
  </states>
  <states id="1472044627757" name="DriveLeft" comment="">
    <plans xsi:type="alica:BehaviourConfiguration">DriveTo.beh#1472045230653</plans>
    <inTransitions>#1472044672299</inTransitions>
    <outTransitions>#1472044674432</outTransitions>
  </states>
  <states id="1472044629029" name="DriveDown" comment="">
    <plans xsi:type="alica:BehaviourConfiguration">DriveTo.beh#1472045313441</plans>
    <inTransitions>#1472044674432</inTransitions>
    <outTransitions>#1472044675779</outTransitions>
  </states>
  <states id="1472044630706" name="DriveRight" comment="">
    <plans xsi:type="alica:BehaviourConfiguration">DriveTo.beh#1472045376619</plans>
    <inTransitions>#1472044675779</inTransitions>
    <outTransitions>#1472044676789</outTransitions>
  </states>
  <transitions id="1472044672299" name="MISSING_NAME" comment="up2left: any child success" msg="">
    <preCondition id="1472044674262" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>#1472044615068</inState>
    <outState>#1472044627757</outState>
  </transitions>
  <transitions id="1472044674432" name="MISSING_NAME" comment="left2down: any child success" msg="">
    <preCondition id="1472044675496" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>#1472044627757</inState>
    <outState>#1472044629029</outState>
  </transitions>
  <transitions id="1472044675779" name="MISSING_NAME" comment="down2right: any child success" msg="">
    <preCondition id="1472044676612" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>#1472044629029</inState>
    <outState>#1472044630706</outState>
  </transitions>
  <transitions id="1472044676789" name="MISSING_NAME" comment="right2up: any child success" msg="">
    <preCondition id="1472044677815" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>#1472044630706</inState>
    <outState>#1472044615068</outState>
  </transitions>
  <entryPoints id="1472044615069" name="MISSING_NAME" comment="" successRequired="false" minCardinality="0" maxCardinality="2147483647">
    <task>../../../Misc/taskrepository.tsk#1225112227903</task>
    <state>#1472044615068</state>
  </entryPoints>
</alica:Plan>
