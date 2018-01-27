<?xml version="1.0" encoding="ASCII"?>
<alica:Plan xmi:version="2.0" xmlns:xmi="http://www.omg.org/XMI" xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance" xmlns:alica="http:///de.uni_kassel.vs.cn" id="1513183613607" name="SoftSquarePlan" comment="" masterPlan="false" utilityFunction="" utilityThreshold="0.1" destinationPath="Plans/TestPlans" priority="0.0" minCardinality="0" maxCardinality="2147483647">
  <states id="1513183636299" name="ToRoot" comment="" entryPoint="1513183636300">
    <plans xsi:type="alica:BehaviourConfiguration">../GenericBehaviours/DriveToPoint.beh#1484145325268</plans>
    <outTransitions>#1513184472444</outTransitions>
  </states>
  <states xsi:type="alica:SuccessState" id="1513184447350" name="RootPointReached" comment="">
    <inTransitions>#1513184479643</inTransitions>
  </states>
  <states id="1513184460815" name="Forward" comment="">
    <plans xsi:type="alica:BehaviourConfiguration">../GenericBehaviours/DriveToPoint.beh#1442921078802</plans>
    <inTransitions>#1513184472444</inTransitions>
    <outTransitions>#1513184473908</outTransitions>
  </states>
  <states id="1513184462600" name="Right" comment="">
    <plans xsi:type="alica:BehaviourConfiguration">../GenericBehaviours/DriveToPoint.beh#1484145108476</plans>
    <inTransitions>#1513184473908</inTransitions>
    <outTransitions>#1513184476668</outTransitions>
  </states>
  <states id="1513184464601" name="Backward" comment="">
    <plans xsi:type="alica:BehaviourConfiguration">../GenericBehaviours/DriveToPoint.beh#1484145243859</plans>
    <inTransitions>#1513184476668</inTransitions>
    <outTransitions>#1513184478047</outTransitions>
  </states>
  <states id="1513184467378" name="Left" comment="">
    <plans xsi:type="alica:BehaviourConfiguration">../GenericBehaviours/DriveToPoint.beh#1484145325268</plans>
    <inTransitions>#1513184478047</inTransitions>
    <outTransitions>#1513184479643</outTransitions>
  </states>
  <transitions id="1513184472444" name="MISSING_NAME" comment="Reached 0,0" msg="">
    <preCondition id="1513184473755" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>#1513183636299</inState>
    <outState>#1513184460815</outState>
  </transitions>
  <transitions id="1513184473908" name="MISSING_NAME" comment="Reached 2000,0" msg="">
    <preCondition id="1513184476531" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>#1513184460815</inState>
    <outState>#1513184462600</outState>
  </transitions>
  <transitions id="1513184476668" name="MISSING_NAME" comment="Reached 2000,-2000" msg="">
    <preCondition id="1513184477910" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>#1513184462600</inState>
    <outState>#1513184464601</outState>
  </transitions>
  <transitions id="1513184478047" name="MISSING_NAME" comment="Reached 0,-2000" msg="">
    <preCondition id="1513184479171" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>#1513184464601</inState>
    <outState>#1513184467378</outState>
  </transitions>
  <transitions id="1513184479643" name="MISSING_NAME" comment="" msg="">
    <preCondition id="1513184480596" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>#1513184467378</inState>
    <outState>#1513184447350</outState>
  </transitions>
  <entryPoints id="1513183636300" name="MISSING_NAME" comment="" successRequired="false" minCardinality="0" maxCardinality="2147483647">
    <task>../../Misc/taskrepository.tsk#1225112227903</task>
    <state>#1513183636299</state>
  </entryPoints>
</alica:Plan>
