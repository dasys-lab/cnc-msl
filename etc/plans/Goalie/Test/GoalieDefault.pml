<?xml version="1.0" encoding="ASCII"?>
<alica:Plan xmi:version="2.0" xmlns:xmi="http://www.omg.org/XMI" xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance" xmlns:alica="http:///de.uni_kassel.vs.cn" id="1447254438614" name="GoalieDefault" comment="" masterPlan="false" utilityFunction="" utilityThreshold="0.1" destinationPath="Plans/Goalie/Test" priority="0.0" minCardinality="0" maxCardinality="1">
  <states id="1447254438615" name="DriveToGoal" comment="" entryPoint="1447254438616">
    <plans xsi:type="alica:BehaviourConfiguration">GoalieBehaviours/DriveToGoal.beh#1447863442558</plans>
    <outTransitions>#1447255446546</outTransitions>
  </states>
  <states id="1447255061404" name="WatchBall" comment="">
    <plans xsi:type="alica:BehaviourConfiguration">GoalieBehaviours/BlockBall.beh#1447863463711</plans>
    <plans xsi:type="alica:BehaviourConfiguration">GoalieBehaviours/WatchBall.beh#1447863472667</plans>
    <inTransitions>#1447255446546</inTransitions>
    <inTransitions>#1447256488872</inTransitions>
    <outTransitions>#1447259462162</outTransitions>
  </states>
  <states id="1447256447585" name="DriveToBall" comment="">
    <plans xsi:type="alica:BehaviourConfiguration">GoalieBehaviours/DriveToBall.beh#1447863503279</plans>
    <inTransitions>#1447259462162</inTransitions>
    <outTransitions>#1447256487026</outTransitions>
  </states>
  <states id="1447256481745" name="Pass" comment="">
    <plans xsi:type="alica:BehaviourConfiguration">GoalieBehaviours/KickToDirection.beh#1447863487000</plans>
    <inTransitions>#1447256487026</inTransitions>
    <outTransitions>#1447256488872</outTransitions>
  </states>
  <transitions id="1447255446546" name="MISSING_NAME" comment="any child success" msg="">
    <preCondition id="1447255447830" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>#1447254438615</inState>
    <outState>#1447255061404</outState>
  </transitions>
  <transitions id="1447256487026" name="MISSING_NAME" comment="Ball is reached and in front of the kickplate" msg="">
    <preCondition id="1447256488686" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>#1447256447585</inState>
    <outState>#1447256481745</outState>
  </transitions>
  <transitions id="1447256488872" name="MISSING_NAME" comment="Ball has been played" msg="">
    <preCondition id="1447256495415" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>#1447256481745</inState>
    <outState>#1447255061404</outState>
  </transitions>
  <transitions id="1447259462162" name="MISSING_NAME" comment="Ball may have been blocked and is now resting inside of goal area" msg="">
    <preCondition id="1447259463769" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>#1447255061404</inState>
    <outState>#1447256447585</outState>
  </transitions>
  <entryPoints id="1447254438616" name="Keeper" comment="" successRequired="false" minCardinality="0" maxCardinality="1">
    <task>../../../Misc/taskrepository.tsk#1221754402444</task>
    <state>#1447254438615</state>
  </entryPoints>
</alica:Plan>
