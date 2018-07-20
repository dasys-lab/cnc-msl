<?xml version="1.0" encoding="ASCII"?>
<alica:Plan xmi:version="2.0" xmlns:xmi="http://www.omg.org/XMI" xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance" xmlns:alica="http:///de.uni_kassel.vs.cn" id="1457173681216" name="DefendPlay" comment="" masterPlan="false" utilityFunction="" utilityThreshold="1.0E-7" destinationPath="Plans/GameStrategy/Gameplay" priority="0.0" minCardinality="1" maxCardinality="4">
  <conditions xsi:type="alica:RuntimeCondition" id="1457173948942" name="NewRuntimeCondition" comment="" conditionString="" pluginName="DefaultPlugin">
    <quantifiers xsi:type="alica:ForallAgents" id="1457338144387" name="MISSING_NAME" comment="" scope="1457173768386">
      <sorts>x</sorts>
      <sorts>y</sorts>
    </quantifiers>
    <quantifiers xsi:type="alica:ForallAgents" id="1457338152956" name="MISSING_NAME" comment="" scope="1457173771067">
      <sorts>x</sorts>
      <sorts>y</sorts>
    </quantifiers>
  </conditions>
  <states id="1457173750607" name="Attack" comment="" entryPoint="1457173750608">
    <plans xsi:type="alica:Plan">../../Attack/StandardAttack.pml#1434046634656</plans>
  </states>
  <states id="1457173773492" name="Block" comment="" entryPoint="1457173768386">
    <plans xsi:type="alica:BehaviourConfiguration">../../Behaviours/Pos4Def.beh#1445438204426</plans>
  </states>
  <states id="1457173775213" name="Defend" comment="" entryPoint="1457173771067">
    <plans xsi:type="alica:BehaviourConfiguration">../../Behaviours/Pos4Def.beh#1445438204426</plans>
  </states>
  <entryPoints id="1457173750608" name="Attack" comment="" successRequired="false" minCardinality="1" maxCardinality="1">
    <task>../../../Misc/taskrepository.tsk#1222613952469</task>
    <state>#1457173750607</state>
  </entryPoints>
  <entryPoints id="1457173768386" name="Block" comment="" successRequired="false" minCardinality="0" maxCardinality="2">
    <task>../../../Misc/taskrepository.tsk#1461237765109</task>
    <state>#1457173773492</state>
  </entryPoints>
  <entryPoints id="1457173771067" name="Defend" comment="" successRequired="false" minCardinality="0" maxCardinality="1">
    <task>../../../Misc/taskrepository.tsk#1225115406909</task>
    <state>#1457173775213</state>
  </entryPoints>
</alica:Plan>
