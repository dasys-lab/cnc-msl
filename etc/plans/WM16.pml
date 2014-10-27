<?xml version="1.0" encoding="ASCII"?>
<alica:Plan xmi:version="2.0" xmlns:xmi="http://www.omg.org/XMI" xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance" xmlns:alica="http:///de.uni_kassel.vs.cn" id="1413992564408" name="WM16" comment="" masterPlan="true" utilityFunction="" utilityThreshold="0.1" destinationPath="Plans" priority="0.0" minCardinality="0" maxCardinality="2147483647">
  <states id="1413992564409" name="Init" comment="" entryPoint="1413992564410">
    <plans xsi:type="alica:BehaviourConfiguration">Behaviours/Stop.beh#1413992626194</plans>
    <outTransitions>#1413992575757</outTransitions>
  </states>
  <states id="1413992572149" name="Drive" comment="">
    <plans xsi:type="alica:BehaviourConfiguration">Behaviours/DriveForward.beh#1414427354149</plans>
    <inTransitions>#1413992575757</inTransitions>
  </states>
  <transitions id="1413992575757" name="" comment="" msg="">
    <preCondition id="1413992578046" name="" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>#1413992564409</inState>
    <outState>#1413992572149</outState>
  </transitions>
  <entryPoints id="1413992564410" name="" comment="" successRequired="false" minCardinality="0" maxCardinality="2147483647">
    <task>../Misc/taskrepository.tsk#1225112227903</task>
    <state>#1413992564409</state>
  </entryPoints>
</alica:Plan>
