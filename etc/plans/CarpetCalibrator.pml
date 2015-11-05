<?xml version="1.0" encoding="ASCII"?>
<alica:Plan xmi:version="2.0" xmlns:xmi="http://www.omg.org/XMI" xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance" xmlns:alica="http:///de.uni_kassel.vs.cn" id="1435159127771" name="CarpetCalibrator" comment="" masterPlan="false" utilityFunction="" utilityThreshold="0.1" destinationPath="Plans" priority="0.0" minCardinality="0" maxCardinality="2147483647">
  <states id="1435159127772" name="Stop" comment="" entryPoint="1435159127773">
    <plans xsi:type="alica:BehaviourConfiguration">GenericBehaviours/Stop.beh#1413992626194</plans>
    <inTransitions>#1435159186825</inTransitions>
    <inTransitions>#1436979741738</inTransitions>
    <outTransitions>#1435159170328</outTransitions>
  </states>
  <states id="1435159146705" name="Spin" comment="">
    <plans xsi:type="alica:BehaviourConfiguration">Behaviours/SpinSlowly.beh#1435159282996</plans>
    <inTransitions>#1435159170328</inTransitions>
    <outTransitions>#1435159186825</outTransitions>
    <outTransitions>#1436979719063</outTransitions>
  </states>
  <states id="1436979695093" name="FinishSpin" comment="">
    <inTransitions>#1436979719063</inTransitions>
    <outTransitions>#1436979741738</outTransitions>
  </states>
  <transitions id="1435159170328" name="MISSING_NAME" comment="situation == start" msg="">
    <preCondition id="1435159171807" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>#1435159127772</inState>
    <outState>#1435159146705</outState>
  </transitions>
  <transitions id="1435159186825" name="MISSING_NAME" comment="stop" msg="">
    <preCondition id="1435159188113" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>#1435159146705</inState>
    <outState>#1435159127772</outState>
  </transitions>
  <transitions id="1436979719063" name="MISSING_NAME" comment="anychild success" msg="">
    <preCondition id="1436979720241" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>#1435159146705</inState>
    <outState>#1436979695093</outState>
  </transitions>
  <transitions id="1436979741738" name="MISSING_NAME" comment="stop" msg="">
    <preCondition id="1436979742914" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>#1436979695093</inState>
    <outState>#1435159127772</outState>
  </transitions>
  <entryPoints id="1435159127773" name="MISSING_NAME" comment="" successRequired="false" minCardinality="0" maxCardinality="2147483647">
    <task>../Misc/taskrepository.tsk#1225112227903</task>
    <state>#1435159127772</state>
  </entryPoints>
</alica:Plan>
