<?xml version="1.0" encoding="ASCII"?>
<alica:Plan xmi:version="2.0" xmlns:xmi="http://www.omg.org/XMI" xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance" xmlns:alica="http:///de.uni_kassel.vs.cn" id="1450178655416" name="Duel" comment="" masterPlan="false" utilityFunction="" utilityThreshold="0.1" destinationPath="" priority="0.0" minCardinality="0" maxCardinality="2147483647">
  <states id="1450178655417" name="Duel" comment="" entryPoint="1450178655418">
    <plans xsi:type="alica:BehaviourConfiguration">../Behaviours/Duel.beh#1450178707835</plans>
  </states>
  <entryPoints id="1450178655418" name="MISSING_NAME" comment="" successRequired="false" minCardinality="0" maxCardinality="2147483647">
    <task>../../Misc/taskrepository.tsk#1225112227903</task>
    <state>#1450178655417</state>
  </entryPoints>
</alica:Plan>
