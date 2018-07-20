<?xml version="1.0" encoding="ASCII"?>
<alica:Plan xmi:version="2.0" xmlns:xmi="http://www.omg.org/XMI" xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance" xmlns:alica="http:///de.uni_kassel.vs.cn" id="1455537014534" name="DropBallPositioning" comment="" masterPlan="false" utilityFunction="" utilityThreshold="0.075" destinationPath="Plans/GameStrategy/Other" priority="0.0" minCardinality="1" maxCardinality="4">
  <states id="1455537014535" name="AttackerPos" comment="" entryPoint="1455537014536">
    <plans xsi:type="alica:BehaviourConfiguration">DropBallAttackerPos.beh#1455537879822</plans>
  </states>
  <states id="1455537278849" name="CoverSpace" comment="" entryPoint="1455537247542">
    <plans xsi:type="alica:BehaviourConfiguration">CoverSpace.beh#1455537928849</plans>
  </states>
  <states id="1455537281002" name="CoverSpaceDefensive" comment="" entryPoint="1455537250535">
    <plans xsi:type="alica:BehaviourConfiguration">CoverSpace.beh#1455537979559</plans>
  </states>
  <states id="1455537283938" name="Defend" comment="" entryPoint="1455537253704">
    <plans xsi:type="alica:BehaviourConfiguration">../../Behaviours/BackroomDefence.beh#1454507819086</plans>
  </states>
  <entryPoints id="1455537014536" name="Attack" comment="" successRequired="false" minCardinality="1" maxCardinality="1">
    <task>../../../Misc/taskrepository.tsk#1222613952469</task>
    <state>#1455537014535</state>
  </entryPoints>
  <entryPoints id="1455537247542" name="SupportAttack" comment="" successRequired="false" minCardinality="0" maxCardinality="1">
    <task>../../../Misc/taskrepository.tsk#1225115536468</task>
    <state>#1455537278849</state>
  </entryPoints>
  <entryPoints id="1455537250535" name="SupportDefend" comment="" successRequired="false" minCardinality="0" maxCardinality="1">
    <task>../../../Misc/taskrepository.tsk#1225116131298</task>
    <state>#1455537281002</state>
  </entryPoints>
  <entryPoints id="1455537253704" name="Defend" comment="" successRequired="false" minCardinality="0" maxCardinality="1">
    <task>../../../Misc/taskrepository.tsk#1225115406909</task>
    <state>#1455537283938</state>
  </entryPoints>
</alica:Plan>
