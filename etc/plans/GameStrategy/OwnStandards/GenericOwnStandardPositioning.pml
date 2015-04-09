<?xml version="1.0" encoding="ASCII"?>
<alica:Plan xmi:version="2.0" xmlns:xmi="http://www.omg.org/XMI" xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance" xmlns:alica="http:///de.uni_kassel.vs.cn" id="1428507630593" name="GenericOwnStandardPositioning" comment="" masterPlan="false" utilityFunction="" utilityThreshold="0.1" destinationPath="Plans/GameStrategy/OwnStandards" priority="0.0" minCardinality="0" maxCardinality="2147483647">
  <states id="1428507664502" name="ReceiverPositioning" comment="" entryPoint="1428507664503">
    <plans xsi:type="alica:BehaviourConfiguration">../../GenericBehaviours/StdReceiverPos.beh#1428508056340</plans>
  </states>
  <states id="1428507881899" name="ExecuterPositioning" comment="" entryPoint="1428507727863">
    <plans xsi:type="alica:BehaviourConfiguration">../../GenericBehaviours/StdExecuterPos.beh#1428508127438</plans>
  </states>
  <states id="1428507884379" name="StandardDefendPositioning" comment="" entryPoint="1428507808894">
    <plans xsi:type="alica:BehaviourConfiguration">../../GenericBehaviours/StdStandardDefendPos.beh#1428508259449</plans>
  </states>
  <entryPoints id="1428507664503" name="MISSING_NAME" comment="" successRequired="false" minCardinality="0" maxCardinality="2147483647">
    <task>../../../Misc/taskrepository.tsk#1428507797481</task>
    <state>#1428507664502</state>
  </entryPoints>
  <entryPoints id="1428507727863" name="MISSING_NAME" comment="" successRequired="false" minCardinality="0" maxCardinality="2147483647">
    <task>../../../Misc/taskrepository.tsk#1428507765874</task>
    <state>#1428507881899</state>
  </entryPoints>
  <entryPoints id="1428507808894" name="NewEntryPoint" comment="" successRequired="false" minCardinality="0" maxCardinality="2147483647">
    <task>../../../Misc/taskrepository.tsk#1225112227903</task>
    <state>#1428507884379</state>
  </entryPoints>
</alica:Plan>
