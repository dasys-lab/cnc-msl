<?xml version="1.0" encoding="ASCII"?>
<alica:Plan xmi:version="2.0" xmlns:xmi="http://www.omg.org/XMI" xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance" xmlns:alica="http:///de.uni_kassel.vs.cn" id="1456247887332" name="GrabBall" comment="" masterPlan="false" utilityFunction="" utilityThreshold="0.1" destinationPath="Plans/Example" priority="0.0" minCardinality="0" maxCardinality="2147483647">
  <states id="1456247900360" name="GetBall" comment="" entryPoint="1456247900361">
    <plans xsi:type="alica:BehaviourConfiguration">../Behaviours/GetBall.beh#1414840399972</plans>
    <outTransitions>#1456247929657</outTransitions>
  </states>
  <states xsi:type="alica:SuccessState" id="1456247926608" name="NewSuccessState" comment="">
    <inTransitions>#1456247929657</inTransitions>
  </states>
  <transitions id="1456247929657" name="MISSING_NAME" comment="" msg="">
    <preCondition id="1456247931188" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>#1456247900360</inState>
    <outState>#1456247926608</outState>
  </transitions>
  <entryPoints id="1456247900361" name="MISSING_NAME" comment="" successRequired="false" minCardinality="0" maxCardinality="2147483647">
    <task>../../Misc/taskrepository.tsk#1225112227903</task>
    <state>#1456247900360</state>
  </entryPoints>
</alica:Plan>
