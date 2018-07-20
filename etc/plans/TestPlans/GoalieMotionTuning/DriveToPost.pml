<?xml version="1.0" encoding="ASCII"?>
<alica:Plan xmi:version="2.0" xmlns:xmi="http://www.omg.org/XMI" xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance" xmlns:alica="http:///de.uni_kassel.vs.cn" id="1464189637940" name="DriveToPost" comment="" masterPlan="true" utilityFunction="" utilityThreshold="0.1" destinationPath="Plans/TestPlans/GoalieMotionTuning" priority="0.0" minCardinality="0" maxCardinality="2147483647">
  <states id="1464189637941" name="Stop" comment="" entryPoint="1464189637942">
    <plans xsi:type="alica:BehaviourConfiguration">../../GenericBehaviours/Stop.beh#1413992626194</plans>
    <inTransitions>#1464979937235</inTransitions>
    <outTransitions>#1464189725286</outTransitions>
  </states>
  <states id="1464189715889" name="DriveToGoal" comment="">
    <plans xsi:type="alica:BehaviourConfiguration">../../Goalie/Test/GoalieBehaviours/DriveToGoal.beh#1447863442558</plans>
    <inTransitions>#1464189725286</inTransitions>
    <outTransitions>#1464189737293</outTransitions>
  </states>
  <states id="1464189729245" name="DriveToPost" comment="">
    <plans xsi:type="alica:BehaviourConfiguration">DriveToPost.beh#1464189840525</plans>
    <inTransitions>#1464189737293</inTransitions>
    <outTransitions>#1464979937235</outTransitions>
  </states>
  <transitions id="1464189725286" name="MISSING_NAME" comment="situation == start" msg="">
    <preCondition id="1464189727040" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>#1464189637941</inState>
    <outState>#1464189715889</outState>
  </transitions>
  <transitions id="1464189737293" name="MISSING_NAME" comment="situation == anychild success" msg="">
    <preCondition id="1464189738685" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>#1464189715889</inState>
    <outState>#1464189729245</outState>
  </transitions>
  <transitions id="1464979937235" name="MISSING_NAME" comment="situation == stop" msg="">
    <preCondition id="1464979938671" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>#1464189729245</inState>
    <outState>#1464189637941</outState>
  </transitions>
  <entryPoints id="1464189637942" name="Keeper" comment="" successRequired="false" minCardinality="0" maxCardinality="2147483647">
    <task>../../../Misc/taskrepository.tsk#1221754402444</task>
    <state>#1464189637941</state>
  </entryPoints>
</alica:Plan>
