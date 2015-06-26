<?xml version="1.0" encoding="ASCII"?>
<alica:Plan xmi:version="2.0" xmlns:xmi="http://www.omg.org/XMI" xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance" xmlns:alica="http:///de.uni_kassel.vs.cn" id="1433763414976" name="CalbrateQualityOfService" comment="" masterPlan="false" utilityFunction="" utilityThreshold="0.1" destinationPath="Plans/ActuatorTest" priority="0.0" minCardinality="0" maxCardinality="2147483647">
  <states id="1435319999213" name="NewState" comment="" entryPoint="1433763414978">
    <plans xsi:type="alica:BehaviourConfiguration">../Behaviours/CalibrateQualityOfService.beh#1435320094002</plans>
  </states>
  <entryPoints id="1433763414978" name="MISSING_NAME" comment="" successRequired="false" minCardinality="0" maxCardinality="2147483647">
    <task>../../Misc/taskrepository.tsk#1225112227903</task>
    <state>#1435319999213</state>
  </entryPoints>
</alica:Plan>
