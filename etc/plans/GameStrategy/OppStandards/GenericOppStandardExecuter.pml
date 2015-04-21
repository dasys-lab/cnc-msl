<?xml version="1.0" encoding="ASCII"?>
<alica:Plan xmi:version="2.0" xmlns:xmi="http://www.omg.org/XMI" xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance" xmlns:alica="http:///de.uni_kassel.vs.cn" id="1429109528736" name="GenericOppStandardExecuter" comment="" masterPlan="false" utilityFunction="" utilityThreshold="0.1" destinationPath="" priority="0.0" minCardinality="0" maxCardinality="2147483647">
  <states id="1429109572408" name="BlockerExecution" comment="" entryPoint="1429109572409">
    <plans xsi:type="alica:BehaviourConfiguration">../../GenericBehaviours/Stop.beh#1413992626194</plans>
  </states>
  <states id="1429109804215" name="StandardDefendExecution" comment="" entryPoint="1429109634871">
    <plans xsi:type="alica:BehaviourConfiguration">../../GenericBehaviours/Stop.beh#1413992626194</plans>
  </states>
  <states id="1429109805827" name="WatcherExecution" comment="" entryPoint="1429109772867">
    <plans xsi:type="alica:BehaviourConfiguration">../../GenericBehaviours/Stop.beh#1413992626194</plans>
  </states>
  <entryPoints id="1429109572409" name="Blocker" comment="" successRequired="false" minCardinality="0" maxCardinality="2147483647">
    <task>../../../Misc/taskrepository.tsk#1429109227638</task>
    <state>#1429109572408</state>
  </entryPoints>
  <entryPoints id="1429109634871" name="MISSING_NAME" comment="" successRequired="false" minCardinality="0" maxCardinality="2147483647">
    <task>../../../Misc/taskrepository.tsk#1429111464037</task>
    <state>#1429109804215</state>
  </entryPoints>
  <entryPoints id="1429109772867" name="Watcher" comment="" successRequired="false" minCardinality="0" maxCardinality="2147483647">
    <task>../../../Misc/taskrepository.tsk#1429108494059</task>
    <state>#1429109805827</state>
  </entryPoints>
</alica:Plan>
