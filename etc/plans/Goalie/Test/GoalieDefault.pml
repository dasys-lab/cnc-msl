<?xml version="1.0" encoding="ASCII"?>
<alica:Plan xmi:version="2.0" xmlns:xmi="http://www.omg.org/XMI" xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance" xmlns:alica="http:///de.uni_kassel.vs.cn" id="1447254438614" name="GoalieDefault" comment="" destinationPath="Plans/Goalie/Test" priority="0.0" minCardinality="0" maxCardinality="1" masterPlan="false" utilityFunction="" utilityThreshold="0.1">
  <states id="1447254438615" name="DriveToGoal" comment="" entryPoint="1447254438616">
    <plans xsi:type="alica:Behaviour">Goalie/Test/GoalieBehaviours/DriveToGoal.beh#1447863424939</plans>
    <outTransitions>Goalie/Test/GoalieDefault.pml#1447255446546</outTransitions>
  </states>
  <states id="1447255061404" name="WatchBall" comment="">
    <plans xsi:type="alica:Behaviour">Behaviours/GoalieExtension.beh#1459249216387</plans>
    <plans xsi:type="alica:Behaviour">Goalie/Test/GoalieBehaviours/WatchBall.beh#1447863466691</plans>
    <inTransitions>Goalie/Test/GoalieDefault.pml#1447255446546</inTransitions>
  </states>
  <transitions id="1447255446546" name="MISSING_NAME" comment="situation == Goalie inside GoalArea" msg="">
    <preCondition id="1447255447830" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>Goalie/Test/GoalieDefault.pml#1447254438615</inState>
    <outState>Goalie/Test/GoalieDefault.pml#1447255061404</outState>
  </transitions>
  <entryPoints id="1447254438616" name="Keeper" comment="" successRequired="false" minCardinality="0" maxCardinality="1">
    <task>taskrepository.tsk#1221754402444</task>
    <state>Goalie/Test/GoalieDefault.pml#1447254438615</state>
  </entryPoints>
</alica:Plan>
