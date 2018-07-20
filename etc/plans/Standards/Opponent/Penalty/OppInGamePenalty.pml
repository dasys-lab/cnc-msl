<?xml version="1.0" encoding="ASCII"?>
<alica:Plan xmi:version="2.0" xmlns:xmi="http://www.omg.org/XMI" xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance" xmlns:alica="http:///de.uni_kassel.vs.cn" id="1466968232004" name="OppInGamePenalty" comment="" masterPlan="false" utilityFunction="" utilityThreshold="0.1" destinationPath="Plans/Standards/Opponent/Penalty" priority="0.0" minCardinality="0" maxCardinality="2147483647">
  <conditions xsi:type="alica:RuntimeCondition" id="1466975666362" name="NewRuntimeCondition" comment="" conditionString="" pluginName="DefaultPlugin">
    <quantifiers xsi:type="alica:ForallAgents" id="1466975671584" name="MISSING_NAME" comment="" scope="1466975645495">
      <sorts>x</sorts>
      <sorts>y</sorts>
    </quantifiers>
  </conditions>
  <states id="1466968232005" name="Pos4Rebounce" comment="" entryPoint="1466968232006">
    <plans xsi:type="alica:BehaviourConfiguration">../../Own/Penalty/InGame/Pos4PenaltyRebounce.beh#1466975991599</plans>
  </states>
  <states id="1466975612430" name="Pos4Intercept" comment="" entryPoint="1466975602577">
    <plans xsi:type="alica:BehaviourConfiguration">Pos4OppPenaltyIntercept.beh#1466975764775</plans>
  </states>
  <states id="1466975637792" name="Block" comment="" entryPoint="1466975645495">
    <plans xsi:type="alica:BehaviourConfiguration">../../../Behaviours/Pos4Def.beh#1445438204426</plans>
  </states>
  <entryPoints id="1466968232006" name="AttackSupport" comment="" successRequired="false" minCardinality="0" maxCardinality="2147483647">
    <task>../../../../Misc/taskrepository.tsk#1225115536468</task>
    <state>#1466968232005</state>
  </entryPoints>
  <entryPoints id="1466975602577" name="Attack" comment="" successRequired="false" minCardinality="0" maxCardinality="2147483647">
    <task>../../../../Misc/taskrepository.tsk#1222613952469</task>
    <state>#1466975612430</state>
  </entryPoints>
  <entryPoints id="1466975645495" name="Block" comment="" successRequired="false" minCardinality="0" maxCardinality="2147483647">
    <task>../../../../Misc/taskrepository.tsk#1461237765109</task>
    <state>#1466975637792</state>
  </entryPoints>
</alica:Plan>
