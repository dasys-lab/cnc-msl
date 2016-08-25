<?xml version="1.0" encoding="ASCII"?>
<alica:Plan xmi:version="2.0" xmlns:xmi="http://www.omg.org/XMI" xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance" xmlns:alica="http:///de.uni_kassel.vs.cn" id="1458553921358" name="WanderPlan" comment="" masterPlan="false" utilityFunction="" utilityThreshold="0.1" destinationPath="Plans/GameStrategy/Other" priority="0.0" minCardinality="1" maxCardinality="2147483647">
  <states id="1458553921359" name="Search4Ball" comment="" entryPoint="1458553921360">
    <plans xsi:type="alica:BehaviourConfiguration">../../Attack/Wander.beh#1434716230628</plans>
  </states>
  <states id="1458553960488" name="SupportSearch4Ball" comment="" entryPoint="1458553931557">
    <plans xsi:type="alica:BehaviourConfiguration">../../Attack/Wander.beh#1434716230628</plans>
  </states>
  <states id="1458553962697" name="Wait4Ball" comment="" entryPoint="1458553934246">
    <plans xsi:type="alica:BehaviourConfiguration">../../GenericBehaviours/Stop.beh#1413992626194</plans>
  </states>
  <states id="1458553964618" name="Wait4Ball" comment="" entryPoint="1458553937231">
    <plans xsi:type="alica:BehaviourConfiguration">../../GenericBehaviours/Stop.beh#1413992626194</plans>
  </states>
  <entryPoints id="1458553921360" name="Attack" comment="" successRequired="false" minCardinality="1" maxCardinality="1">
    <task>../../../Misc/taskrepository.tsk#1222613952469</task>
    <state>#1458553921359</state>
  </entryPoints>
  <entryPoints id="1458553931557" name="ReceivePassInGame" comment="" successRequired="false" minCardinality="0" maxCardinality="1">
    <task>../../../Misc/taskrepository.tsk#1307185798142</task>
    <state>#1458553960488</state>
  </entryPoints>
  <entryPoints id="1458553934246" name="Block" comment="" successRequired="false" minCardinality="0" maxCardinality="2147483647">
    <task>../../../Misc/taskrepository.tsk#1461237765109</task>
    <state>#1458553962697</state>
  </entryPoints>
  <entryPoints id="1458553937231" name="Defend" comment="" successRequired="false" minCardinality="0" maxCardinality="2147483647">
    <task>../../../Misc/taskrepository.tsk#1225115406909</task>
    <state>#1458553964618</state>
  </entryPoints>
</alica:Plan>
