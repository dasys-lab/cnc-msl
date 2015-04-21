<?xml version="1.0" encoding="ASCII"?>
<alica:Plan xmi:version="2.0" xmlns:xmi="http://www.omg.org/XMI" xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance" xmlns:alica="http:///de.uni_kassel.vs.cn" id="1429108230432" name="GenericOppStandardPositioning" comment="" masterPlan="false" utilityFunction="" utilityThreshold="0.1" destinationPath="Plans/GameStrategy/OppStandards" priority="0.0" minCardinality="0" maxCardinality="2147483647">
  <states id="1429108281685" name="WatcherPositioning" comment="" entryPoint="1429108491532">
    <plans xsi:type="alica:BehaviourConfiguration">../../GenericBehaviours/StandardWatcherPositioningDefault.beh#1429109434270</plans>
  </states>
  <states id="1429108283427" name="BlockerPositioning" comment="" entryPoint="1429109224669">
    <plans xsi:type="alica:BehaviourConfiguration">../../GenericBehaviours/StandardBlockerPositioning.beh#1429109488432</plans>
  </states>
  <states id="1429110479804" name="StandardDefend" comment="" entryPoint="1429110470504">
    <plans xsi:type="alica:BehaviourConfiguration">../../GenericBehaviours/StandardStdDefendPositioning.beh#1429110549548</plans>
  </states>
  <entryPoints id="1429108491532" name="MISSING_NAME" comment="" successRequired="false" minCardinality="0" maxCardinality="1">
    <task>../../../Misc/taskrepository.tsk#1429108494059</task>
    <state>#1429108281685</state>
  </entryPoints>
  <entryPoints id="1429109224669" name="MISSING_NAME" comment="" successRequired="false" minCardinality="0" maxCardinality="2147483647">
    <task>../../../Misc/taskrepository.tsk#1429109227638</task>
    <state>#1429108283427</state>
  </entryPoints>
  <entryPoints id="1429110470504" name="StandardDefend" comment="" successRequired="false" minCardinality="0" maxCardinality="2147483647">
    <task>../../../Misc/taskrepository.tsk#1429111464037</task>
    <state>#1429110479804</state>
  </entryPoints>
</alica:Plan>
