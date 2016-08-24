<?xml version="1.0" encoding="ASCII"?>
<alica:Plan xmi:version="2.0" xmlns:xmi="http://www.omg.org/XMI" xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance" xmlns:alica="http:///de.uni_kassel.vs.cn" id="1472044221834" name="DriveInSquareMaster" comment="" masterPlan="true" utilityFunction="" utilityThreshold="0.1" destinationPath="Plans/TestPlans/DriveInSquare" priority="0.0" minCardinality="0" maxCardinality="2147483647">
  <states id="1472044221835" name="Stop" comment="Stop" entryPoint="1472044221836">
    <plans xsi:type="alica:BehaviourConfiguration">../../GenericBehaviours/Stop.beh#1413992626194</plans>
    <inTransitions>#1472044412610</inTransitions>
    <outTransitions>#1472044410951</outTransitions>
  </states>
  <states id="1472044397035" name="Start" comment="start">
    <plans xsi:type="alica:Plan">DriveInSquare.pml#1472044587219</plans>
    <inTransitions>#1472044410951</inTransitions>
    <outTransitions>#1472044412610</outTransitions>
  </states>
  <transitions id="1472044410951" name="MISSING_NAME" comment="stop2start" msg="">
    <preCondition id="1472044412383" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>#1472044221835</inState>
    <outState>#1472044397035</outState>
  </transitions>
  <transitions id="1472044412610" name="MISSING_NAME" comment="start2stop" msg="">
    <preCondition id="1472044413612" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>#1472044397035</inState>
    <outState>#1472044221835</outState>
  </transitions>
  <entryPoints id="1472044221836" name="MISSING_NAME" comment="" successRequired="false" minCardinality="0" maxCardinality="2147483647">
    <task>../../../Misc/taskrepository.tsk#1225112227903</task>
    <state>#1472044221835</state>
  </entryPoints>
</alica:Plan>
