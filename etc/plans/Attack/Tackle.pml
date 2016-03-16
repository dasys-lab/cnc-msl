<?xml version="1.0" encoding="ASCII"?>
<alica:Plan xmi:version="2.0" xmlns:xmi="http://www.omg.org/XMI" xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance" xmlns:alica="http:///de.uni_kassel.vs.cn" id="1434116965565" name="Tackle" comment="" masterPlan="false" utilityFunction="" utilityThreshold="0.1" destinationPath="Plans/Attack" priority="0.0" minCardinality="0" maxCardinality="2147483647">
  <states id="1434116965566" name="Tackle" comment="" entryPoint="1434116965567">
    <plans xsi:type="alica:BehaviourConfiguration">Tackle.beh#1434807680165</plans>
  </states>
  <entryPoints id="1434116965567" name="Attack" comment="" successRequired="false" minCardinality="0" maxCardinality="2147483647">
    <task>../../Misc/taskrepository.tsk#1222613952469</task>
    <state>#1434116965566</state>
  </entryPoints>
</alica:Plan>
