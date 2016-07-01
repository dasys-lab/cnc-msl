<?xml version="1.0" encoding="ASCII"?>
<alica:Plan xmi:version="2.0" xmlns:xmi="http://www.omg.org/XMI" xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance" xmlns:alica="http:///de.uni_kassel.vs.cn" id="1467396347588" name="RotationCalibration" comment="" masterPlan="true" utilityFunction="" utilityThreshold="0.1" destinationPath="Plans/Calibration" priority="0.0" minCardinality="0" maxCardinality="2147483647">
  <states id="1467396347589" name="Stop" comment="" entryPoint="1467396347590">
    <inTransitions>#1467396705635</inTransitions>
    <outTransitions>#1467396616225</outTransitions>
  </states>
  <states id="1467396438734" name="Rotating" comment="">
    <plans xsi:type="alica:BehaviourConfiguration">RotateOnce.beh#1467398000539</plans>
    <inTransitions>#1467396616225</inTransitions>
    <outTransitions>#1467396705635</outTransitions>
  </states>
  <transitions id="1467396616225" name="MISSING_NAME" comment="Status Start?" msg="">
    <preCondition id="1467396619848" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>#1467396347589</inState>
    <outState>#1467396438734</outState>
  </transitions>
  <transitions id="1467396705635" name="MISSING_NAME" comment="Rotation done? Or Status Stop?" msg="">
    <preCondition id="1467396709878" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>#1467396438734</inState>
    <outState>#1467396347589</outState>
  </transitions>
  <entryPoints id="1467396347590" name="MISSING_NAME" comment="" successRequired="false" minCardinality="0" maxCardinality="2147483647">
    <task>../../Misc/taskrepository.tsk#1225112227903</task>
    <state>#1467396347589</state>
  </entryPoints>
</alica:Plan>
