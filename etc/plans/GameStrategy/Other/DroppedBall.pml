<?xml version="1.0" encoding="ASCII"?>
<alica:Plan xmi:version="2.0" xmlns:xmi="http://www.omg.org/XMI" xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance" xmlns:alica="http:///de.uni_kassel.vs.cn" id="1426694906399" name="DroppedBall" comment="" destinationPath="Plans/GameStrategy/Other" priority="0.0" minCardinality="0" maxCardinality="2147483647" masterPlan="false" utilityFunction="" utilityThreshold="0.1">
  <states id="1426694906400" name="DropBallType" comment="" entryPoint="1426694906401">
    <plans xsi:type="alica:PlanType">GameStrategy/Other/DropBallType.pty#1426696396136</plans>
    <outTransitions>GameStrategy/Other/DroppedBall.pml#1426696476530</outTransitions>
  </states>
  <states xsi:type="alica:SuccessState" id="1426696458333" name="NewSuccessState" comment="">
    <inTransitions>GameStrategy/Other/DroppedBall.pml#1426696476530</inTransitions>
  </states>
  <transitions id="1426696476530" name="MISSING_NAME" comment="anychildsuccess" msg="">
    <preCondition id="1426696478377" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>GameStrategy/Other/DroppedBall.pml#1426694906400</inState>
    <outState>GameStrategy/Other/DroppedBall.pml#1426696458333</outState>
  </transitions>
  <entryPoints id="1426694906401" name="MISSING_NAME" comment="" successRequired="true" minCardinality="0" maxCardinality="2147483647">
    <task>taskrepository.tsk#1225112227903</task>
    <state>GameStrategy/Other/DroppedBall.pml#1426694906400</state>
  </entryPoints>
</alica:Plan>
