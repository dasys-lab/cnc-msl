<?xml version="1.0" encoding="ASCII"?>
<alica:Plan xmi:version="2.0" xmlns:xmi="http://www.omg.org/XMI" xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance" xmlns:alica="http:///de.uni_kassel.vs.cn" id="1437902649389" name="AttackOppGoalPlan" comment="" masterPlan="false" utilityFunction="" utilityThreshold="0.1" destinationPath="Plans/Attack/TestPlans" priority="0.0" minCardinality="0" maxCardinality="2147483647">
  <states id="1437902693350" name="DribbleToOppPenaltySpot" comment="">
    <plans xsi:type="alica:BehaviourConfiguration">../DribbleToAttackPoint.beh#1436855860607</plans>
    <plans xsi:type="alica:BehaviourConfiguration">../../Behaviours/ShovelSelect.beh#1435156714286</plans>
  </states>
  <states id="1437902704803" name="DribbleToOwnPenaltySpot" comment="">
    <plans xsi:type="alica:BehaviourConfiguration">../DribbleToAttackPoint.beh#1437391438054</plans>
  </states>
  <states id="1437902762624" name="Shoot" comment="">
    <plans xsi:type="alica:BehaviourConfiguration">../../Behaviours/GoalKick.beh#1415205578139</plans>
  </states>
  <entryPoints id="1437902649391" name="MISSING_NAME" comment="" successRequired="true" minCardinality="0" maxCardinality="2147483647">
    <task>../../../Misc/taskrepository.tsk#1225112227903</task>
  </entryPoints>
</alica:Plan>
