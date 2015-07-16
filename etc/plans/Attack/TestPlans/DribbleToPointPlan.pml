<?xml version="1.0" encoding="ASCII"?>
<alica:Plan xmi:version="2.0" xmlns:xmi="http://www.omg.org/XMI" xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance" xmlns:alica="http:///de.uni_kassel.vs.cn" id="1436960829485" name="DribbleToPointPlan" comment="" masterPlan="false" utilityFunction="" utilityThreshold="0.1" destinationPath="Plans/Attack/TestPlans" priority="0.0" minCardinality="0" maxCardinality="2147483647">
  <states id="1436960962060" name="DribbleToOppPenaltySpot" comment="" entryPoint="1436960854733">
    <plans xsi:type="alica:BehaviourConfiguration">../DribbleToAttackPoint.beh#1436855860607</plans>
  </states>
  <entryPoints id="1436960854733" name="MISSING_NAME" comment="" successRequired="false" minCardinality="0" maxCardinality="2147483647">
    <task>../../../Misc/taskrepository.tsk#1225112227903</task>
    <state>#1436960962060</state>
  </entryPoints>
</alica:Plan>
