<?xml version="1.0" encoding="ASCII"?>
<alica:Plan xmi:version="2.0" xmlns:xmi="http://www.omg.org/XMI" xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance" xmlns:alica="http:///de.uni_kassel.vs.cn" id="1417621468963" name="TwoHoledWallMaster" comment="" masterPlan="true" utilityFunction="" utilityThreshold="0.1" destinationPath="Plans/TwoHoledWall" priority="0.0" minCardinality="0" maxCardinality="2147483647">
  <states id="1417621468964" name="Stop" comment="" entryPoint="1417621468965">
    <plans xsi:type="alica:BehaviourConfiguration">../GenericBehaviours/Stop.beh#1413992626194</plans>
    <inTransitions>#1422464874668</inTransitions>
    <inTransitions>#1496753329785</inTransitions>
    <outTransitions>#1422464819985</outTransitions>
    <outTransitions>#1496753328478</outTransitions>
  </states>
  <states id="1422464768858" name="Joysick" comment="">
    <plans xsi:type="alica:BehaviourConfiguration">../Behaviours/Joystick.beh#1421854995808</plans>
    <inTransitions>#1422464819985</inTransitions>
    <outTransitions>#1422464874668</outTransitions>
  </states>
  <states id="1496753132907" name="Start" comment="">
    <plans xsi:type="alica:Plan">TwoHoledWallAlign.pml#1496753106611</plans>
    <plans xsi:type="alica:BehaviourConfiguration">DribbleConstTwoHoledWall.beh#1496840164871</plans>
    <inTransitions>#1496753328478</inTransitions>
    <outTransitions>#1496753329785</outTransitions>
  </states>
  <transitions id="1422464819985" name="MISSING_NAME" comment="situation == joystick" msg="">
    <preCondition id="1422464821945" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>#1417621468964</inState>
    <outState>#1422464768858</outState>
  </transitions>
  <transitions id="1422464874668" name="MISSING_NAME" comment="situation != joystick" msg="">
    <preCondition id="1422464875980" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>#1422464768858</inState>
    <outState>#1417621468964</outState>
  </transitions>
  <transitions id="1496753328478" name="MISSING_NAME" comment="situation == start" msg="">
    <preCondition id="1496753329486" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>#1417621468964</inState>
    <outState>#1496753132907</outState>
  </transitions>
  <transitions id="1496753329785" name="MISSING_NAME" comment="situation == stop" msg="">
    <preCondition id="1496753330441" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>#1496753132907</inState>
    <outState>#1417621468964</outState>
  </transitions>
  <entryPoints id="1417621468965" name="MISSING_NAME" comment="" successRequired="false" minCardinality="0" maxCardinality="2147483647">
    <task>../../Misc/taskrepository.tsk#1225112227903</task>
    <state>#1417621468964</state>
  </entryPoints>
</alica:Plan>
