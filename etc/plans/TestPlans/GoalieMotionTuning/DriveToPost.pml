<?xml version="1.0" encoding="ASCII"?>
<alica:Plan xmi:version="2.0" xmlns:xmi="http://www.omg.org/XMI" xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance" xmlns:alica="http:///de.uni_kassel.vs.cn" id="1464189637940" name="DriveToPost" comment="" destinationPath="Plans/TestPlans/GoalieMotionTuning" priority="0.0" minCardinality="0" maxCardinality="2147483647" masterPlan="false" utilityFunction="" utilityThreshold="0.1">
  <states id="1464189637941" name="Stop" comment="" entryPoint="1464189637942">
    <plans xsi:type="alica:Behaviour">GenericBehaviours/Stop.beh#1413992604875</plans>
    <inTransitions>TestPlans/GoalieMotionTuning/DriveToPost.pml#1464979937235</inTransitions>
    <outTransitions>TestPlans/GoalieMotionTuning/DriveToPost.pml#1464189725286</outTransitions>
  </states>
  <states id="1464189715889" name="DriveToGoal" comment="">
    <plans xsi:type="alica:Behaviour">Goalie/Test/GoalieBehaviours/DriveToGoal.beh#1447863424939</plans>
    <inTransitions>TestPlans/GoalieMotionTuning/DriveToPost.pml#1464189725286</inTransitions>
    <outTransitions>TestPlans/GoalieMotionTuning/DriveToPost.pml#1464189737293</outTransitions>
  </states>
  <states id="1464189729245" name="DriveToPost" comment="">
    <plans xsi:type="alica:Behaviour">TestPlans/GoalieMotionTuning/DriveToPost.beh#1464189819779</plans>
    <inTransitions>TestPlans/GoalieMotionTuning/DriveToPost.pml#1464189737293</inTransitions>
    <outTransitions>TestPlans/GoalieMotionTuning/DriveToPost.pml#1464979937235</outTransitions>
  </states>
  <transitions id="1464189725286" name="MISSING_NAME" comment="situation == start" msg="">
    <preCondition id="1464189727040" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>TestPlans/GoalieMotionTuning/DriveToPost.pml#1464189637941</inState>
    <outState>TestPlans/GoalieMotionTuning/DriveToPost.pml#1464189715889</outState>
  </transitions>
  <transitions id="1464189737293" name="MISSING_NAME" comment="situation == anychild success" msg="">
    <preCondition id="1464189738685" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>TestPlans/GoalieMotionTuning/DriveToPost.pml#1464189715889</inState>
    <outState>TestPlans/GoalieMotionTuning/DriveToPost.pml#1464189729245</outState>
  </transitions>
  <transitions id="1464979937235" name="MISSING_NAME" comment="situation == stop" msg="">
    <preCondition id="1464979938671" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>TestPlans/GoalieMotionTuning/DriveToPost.pml#1464189729245</inState>
    <outState>TestPlans/GoalieMotionTuning/DriveToPost.pml#1464189637941</outState>
  </transitions>
  <entryPoints id="1464189637942" name="Keeper" comment="" successRequired="false" minCardinality="0" maxCardinality="2147483647">
    <task>taskrepository.tsk#1221754402444</task>
    <state>TestPlans/GoalieMotionTuning/DriveToPost.pml#1464189637941</state>
  </entryPoints>
</alica:Plan>
