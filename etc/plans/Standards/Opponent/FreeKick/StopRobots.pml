<?xml version="1.0" encoding="ASCII"?>
<alica:Plan xmi:version="2.0" xmlns:xmi="http://www.omg.org/XMI" xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance" xmlns:alica="http:///de.uni_kassel.vs.cn" id="1457015643757" name="StopRobots" comment="" destinationPath="Plans/Standards/Opponent/FreeKick" priority="0.0" minCardinality="0" maxCardinality="2147483647" masterPlan="false" utilityFunction="" utilityThreshold="0.1">
  <states id="1457015643758" name="Stop" comment="" entryPoint="1457015643759">
    <plans xsi:type="alica:Behaviour">GenericBehaviours/Stop.beh#1413992604875</plans>
  </states>
  <entryPoints id="1457015643759" name="MISSING_NAME" comment="" successRequired="true" minCardinality="0" maxCardinality="2147483647">
    <task>taskrepository.tsk#1225112227903</task>
    <state>Standards/Opponent/FreeKick/StopRobots.pml#1457015643758</state>
  </entryPoints>
</alica:Plan>
