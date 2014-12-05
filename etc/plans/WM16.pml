<?xml version="1.0" encoding="ASCII"?>
<alica:Plan xmi:version="2.0" xmlns:xmi="http://www.omg.org/XMI" xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance" xmlns:alica="http:///de.uni_kassel.vs.cn" id="1413992564408" name="WM16" comment="" masterPlan="true" utilityFunction="" utilityThreshold="0.1" destinationPath="Plans" priority="0.0" minCardinality="0" maxCardinality="2147483647">
  <states id="1413992564409" name="Init" comment="" entryPoint="1413992564410">
    <plans xsi:type="alica:BehaviourConfiguration">GenericBehaviours/Stop.beh#1413992626194</plans>
    <outTransitions>#1413992575757</outTransitions>
  </states>
  <states id="1413992572149" name="Drive" comment="">
    <plans xsi:type="alica:BehaviourConfiguration">Behaviours/GetBall.beh#1414840399972</plans>
    <inTransitions>#1413992575757</inTransitions>
    <inTransitions>#1414883776988</inTransitions>
    <inTransitions>#1415207005689</inTransitions>
    <inTransitions>#1415207059261</inTransitions>
    <outTransitions>#1414752349075</outTransitions>
  </states>
  <states id="1414752333556" name="Dribble" comment="">
    <plans xsi:type="alica:BehaviourConfiguration">Behaviours/DribbleToPoint.beh#1414752423981</plans>
    <inTransitions>#1414752349075</inTransitions>
    <outTransitions>#1414883776988</outTransitions>
    <outTransitions>#1415205265272</outTransitions>
  </states>
  <states id="1415205191506" name="AlineToGoal" comment="">
    <plans xsi:type="alica:BehaviourConfiguration">Behaviours/AlineToGoal.beh#1415205285582</plans>
    <inTransitions>#1415205265272</inTransitions>
    <outTransitions>#1415206978594</outTransitions>
    <outTransitions>#1415207005689</outTransitions>
  </states>
  <states id="1415206960125" name="GoalKick" comment="">
    <plans xsi:type="alica:BehaviourConfiguration">Behaviours/GoalKick.beh#1415205578139</plans>
    <inTransitions>#1415206978594</inTransitions>
    <outTransitions>#1415207059261</outTransitions>
  </states>
  <transitions id="1413992575757" name="" comment="" msg="">
    <preCondition id="1413992578046" name="" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>#1413992564409</inState>
    <outState>#1413992572149</outState>
  </transitions>
  <transitions id="1414752349075" name="haveBall" comment="" msg="">
    <preCondition id="1414752354525" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>#1413992572149</inState>
    <outState>#1414752333556</outState>
  </transitions>
  <transitions id="1414883776988" name="lostBall" comment="" msg="">
    <preCondition id="1414883779788" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>#1414752333556</inState>
    <outState>#1413992572149</outState>
  </transitions>
  <transitions id="1415205265272" name="MISSING_NAME" comment="" msg="">
    <preCondition id="1415205268720" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>#1414752333556</inState>
    <outState>#1415205191506</outState>
  </transitions>
  <transitions id="1415206978594" name="MISSING_NAME" comment="" msg="">
    <preCondition id="1415206979578" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>#1415205191506</inState>
    <outState>#1415206960125</outState>
  </transitions>
  <transitions id="1415207005689" name="MISSING_NAME" comment="" msg="">
    <preCondition id="1415207006891" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>#1415205191506</inState>
    <outState>#1413992572149</outState>
  </transitions>
  <transitions id="1415207059261" name="MISSING_NAME" comment="" msg="">
    <preCondition id="1415207067102" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>#1415206960125</inState>
    <outState>#1413992572149</outState>
  </transitions>
  <entryPoints id="1413992564410" name="" comment="" successRequired="false" minCardinality="0" maxCardinality="2147483647">
    <task>../Misc/taskrepository.tsk#1225112227903</task>
    <state>#1413992564409</state>
  </entryPoints>
</alica:Plan>
