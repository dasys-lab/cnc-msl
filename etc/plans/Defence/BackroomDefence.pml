<?xml version="1.0" encoding="ASCII"?>
<alica:Plan xmi:version="2.0" xmlns:xmi="http://www.omg.org/XMI" xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance" xmlns:alica="http:///de.uni_kassel.vs.cn" id="1454507425037" name="BackroomDefence" comment="" masterPlan="false" utilityFunction="" utilityThreshold="0.1" destinationPath="Plans/Defence" priority="0.0" minCardinality="0" maxCardinality="2147483647">
  <states id="1454507425038" name="BackroomDefence" comment="" entryPoint="1454507425039">
    <plans xsi:type="alica:BehaviourConfiguration">../Behaviours/BackroomDefence.beh#1454507819086</plans>
  </states>
  <entryPoints id="1454507425039" name="MISSING_NAME" comment="" successRequired="false" minCardinality="0" maxCardinality="2147483647">
    <task>../../Misc/taskrepository.tsk#1225112227903</task>
    <state>#1454507425038</state>
  </entryPoints>
</alica:Plan>
