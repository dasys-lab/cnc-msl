<?xml version="1.0" encoding="ASCII"?>
<alica:Plan xmi:version="2.0" xmlns:xmi="http://www.omg.org/XMI" xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance" xmlns:alica="http:///de.uni_kassel.vs.cn" id="1413992564408" name="WM16" comment="" masterPlan="true" utilityFunction="" utilityThreshold="0.1" destinationPath="Plans" priority="0.0" minCardinality="0" maxCardinality="2147483647">
  <states id="1413992564409" name="Init" comment="" entryPoint="1413992564410">
    <plans xsi:type="alica:BehaviourConfiguration">Behaviours/Stop.beh#1413992626194</plans>
    <outTransitions>#1413992575757</outTransitions>
  </states>
  <states id="1413992572149" name="Drive" comment="">
    <plans xsi:type="alica:BehaviourConfiguration">Behaviours/DriveForward.beh#1414427354149</plans>
    <inTransitions>#1413992575757</inTransitions>
    <inTransitions>#1414769683132</inTransitions>
    <outTransitions>#1414752349075</outTransitions>
  </states>
  <states id="1414752333556" name="Dribble" comment="">
    <plans xsi:type="alica:BehaviourConfiguration">Behaviours/DribbleToPoint.beh#1414752423981</plans>
    <inTransitions>#1414752349075</inTransitions>
    <outTransitions>#1414769683132</outTransitions>
  </states>
  <transitions id="1413992575757" name="" comment="" msg="">
    <preCondition id="1413992578046" name="" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>#1413992564409</inState>
    <outState>#1413992572149</outState>
  </transitions>
  <transitions id="1414752349075" name="MISSING_NAME" comment="" msg="">
    <preCondition id="1414752354525" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>#1413992572149</inState>
    <outState>#1414752333556</outState>
  </transitions>
  <transitions id="1414769683132" name="MISSING_NAME" comment="" msg="">
    <preCondition id="1414769686605" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>#1414752333556</inState>
    <outState>#1413992572149</outState>
  </transitions>
  <entryPoints id="1413992564410" name="" comment="" successRequired="false" minCardinality="0" maxCardinality="2147483647">
    <task>../Misc/taskrepository.tsk#1225112227903</task>
    <state>#1413992564409</state>
  </entryPoints>
</alica:Plan>
