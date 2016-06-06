<?xml version="1.0" encoding="ASCII"?>
<alica:Plan xmi:version="2.0" xmlns:xmi="http://www.omg.org/XMI" xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance" xmlns:alica="http:///de.uni_kassel.vs.cn" id="1464532006730" name="TestFreeKickOppHalfMaster" comment="" masterPlan="true" utilityFunction="" utilityThreshold="0.1" destinationPath="Plans/Standards/Own/FreeKick/Test" priority="0.0" minCardinality="0" maxCardinality="2147483647">
  <states id="1464532006731" name="FreeKickOppHalf" comment="" entryPoint="1464532006732">
    <plans xsi:type="alica:Plan">../OwnFreeKickInOppHalf.pml#1464531946023</plans>
  </states>
  <entryPoints id="1464532006732" name="MISSING_NAME" comment="" successRequired="false" minCardinality="0" maxCardinality="2147483647">
    <task>../../../../../Misc/taskrepository.tsk#1225112227903</task>
    <state>#1464532006731</state>
  </entryPoints>
</alica:Plan>
