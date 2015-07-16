<?xml version="1.0" encoding="ASCII"?>
<alica:Plan xmi:version="2.0" xmlns:xmi="http://www.omg.org/XMI" xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance" xmlns:alica="http:///de.uni_kassel.vs.cn" id="1436960829485" name="DribbleToPointPlan" comment="" masterPlan="false" utilityFunction="" utilityThreshold="0.1" destinationPath="Plans/Attack/TestPlans" priority="0.0" minCardinality="0" maxCardinality="2147483647">
  <states id="1436960854732" name="DriveToOwnPenaltySpot" comment="" entryPoint="1436960854733">
    <plans xsi:type="alica:BehaviourConfiguration">../../GenericBehaviours/DriveToPoint.beh#1436961206415</plans>
    <outTransitions>#1436960982644</outTransitions>
  </states>
  <states id="1436960880333" name="GrabBall" comment="">
    <plans xsi:type="alica:BehaviourConfiguration">../../GenericBehaviours/InterceptCarefully.beh#1427703234654</plans>
    <inTransitions>#1436960982644</inTransitions>
    <inTransitions>#1436960986557</inTransitions>
    <outTransitions>#1436960984714</outTransitions>
  </states>
  <states id="1436960962060" name="DribbleToOppPenaltySpot" comment="">
    <plans xsi:type="alica:BehaviourConfiguration">../DribbleToAttackPoint.beh#1436855860607</plans>
    <inTransitions>#1436960984714</inTransitions>
    <outTransitions>#1436960986557</outTransitions>
  </states>
  <transitions id="1436960982644" name="MISSING_NAME" comment="any child success" msg="">
    <preCondition id="1436960984355" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>#1436960854732</inState>
    <outState>#1436960880333</outState>
  </transitions>
  <transitions id="1436960984714" name="MISSING_NAME" comment="haveball" msg="">
    <preCondition id="1436960986275" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>#1436960880333</inState>
    <outState>#1436960962060</outState>
  </transitions>
  <transitions id="1436960986557" name="MISSING_NAME" comment="lostBall" msg="">
    <preCondition id="1436960988225" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>#1436960962060</inState>
    <outState>#1436960880333</outState>
  </transitions>
  <entryPoints id="1436960854733" name="MISSING_NAME" comment="" successRequired="false" minCardinality="0" maxCardinality="2147483647">
    <task>../../../Misc/taskrepository.tsk#1225112227903</task>
    <state>#1436960854732</state>
  </entryPoints>
</alica:Plan>
