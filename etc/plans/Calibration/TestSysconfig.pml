<?xml version="1.0" encoding="ASCII"?>
<alica:Plan xmi:version="2.0" xmlns:xmi="http://www.omg.org/XMI" xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance" xmlns:alica="http:///de.uni_kassel.vs.cn" id="1469193490832" name="TestSysconfig" comment="" masterPlan="true" utilityFunction="" utilityThreshold="0.1" destinationPath="Plans/Calibration" priority="0.0" minCardinality="0" maxCardinality="2147483647">
  <states id="1469193490833" name="NewState" comment="" entryPoint="1469193490834">
    <plans xsi:type="alica:BehaviourConfiguration">../Behaviours/TestSysconfig.beh#1469193546696</plans>
  </states>
  <entryPoints id="1469193490834" name="MISSING_NAME" comment="" successRequired="false" minCardinality="0" maxCardinality="2147483647">
    <task>../../Misc/taskrepository.tsk#1225112227903</task>
    <state>#1469193490833</state>
  </entryPoints>
</alica:Plan>
