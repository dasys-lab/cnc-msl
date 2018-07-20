<?xml version="1.0" encoding="ASCII"?>
<alica:Plan xmi:version="2.0" xmlns:xmi="http://www.omg.org/XMI" xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance" xmlns:alica="http:///de.uni_kassel.vs.cn" id="1457173546734" name="GamePlay" comment="" masterPlan="false" utilityFunction="" utilityThreshold="0.1" destinationPath="Plans/GameStrategy/Gameplay" priority="0.0" minCardinality="0" maxCardinality="2147483647">
  <states id="1457173565462" name="Defend" comment="" entryPoint="1457173565463">
    <plans xsi:type="alica:Plan">DefendPlay.pml#1457173681216</plans>
    <inTransitions>#1457173604378</inTransitions>
    <outTransitions>#1457173602825</outTransitions>
  </states>
  <states id="1457173576569" name="Attack" comment="">
    <plans xsi:type="alica:PlanType">GamePlayAttackType.pty#1457173624653</plans>
    <inTransitions>#1457173602825</inTransitions>
    <outTransitions>#1457173604378</outTransitions>
  </states>
  <states id="1457173842758" name="DefendGoal" comment="" entryPoint="1457173833589">
    <plans xsi:type="alica:Plan">../../Goalie/Test/GoalieDefault.pml#1447254438614</plans>
  </states>
  <transitions id="1457173602825" name="MISSING_NAME" comment="GameState == (Attack || Melee) " msg="">
    <preCondition id="1457173604049" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>#1457173565462</inState>
    <outState>#1457173576569</outState>
  </transitions>
  <transitions id="1457173604378" name="MISSING_NAME" comment="GameState == (Defend) " msg="">
    <preCondition id="1457173606067" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>#1457173576569</inState>
    <outState>#1457173565462</outState>
  </transitions>
  <entryPoints id="1457173565463" name="MISSING_NAME" comment="" successRequired="false" minCardinality="0" maxCardinality="2147483647">
    <task>../../../Misc/taskrepository.tsk#1225112227903</task>
    <state>#1457173565462</state>
  </entryPoints>
  <entryPoints id="1457173833589" name="Keeper" comment="" successRequired="false" minCardinality="0" maxCardinality="2147483647">
    <task>../../../Misc/taskrepository.tsk#1221754402444</task>
    <state>#1457173842758</state>
  </entryPoints>
</alica:Plan>
