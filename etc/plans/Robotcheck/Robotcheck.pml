<?xml version="1.0" encoding="ASCII"?>
<alica:Plan xmi:version="2.0" xmlns:xmi="http://www.omg.org/XMI" xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance" xmlns:alica="http:///de.uni_kassel.vs.cn" id="1456756058055" name="Robotcheck" comment="" masterPlan="false" utilityFunction="" utilityThreshold="0.1" destinationPath="Plans/Robotcheck" priority="0.0" minCardinality="0" maxCardinality="2147483647">
  <states id="1456756058056" name="RobotCheck" comment="" entryPoint="1456756058057">
    <plans xsi:type="alica:BehaviourConfiguration">RobotTest.beh#1456756164754</plans>
  </states>
  <entryPoints id="1456756058057" name="MISSING_NAME" comment="" successRequired="false" minCardinality="0" maxCardinality="2147483647">
    <task>../../Misc/taskrepository.tsk#1225112227903</task>
    <state>#1456756058056</state>
  </entryPoints>
</alica:Plan>
