<?xml version="1.0" encoding="ASCII"?>
<alica:Plan xmi:version="2.0" xmlns:xmi="http://www.omg.org/XMI" xmlns:alica="http:///de.uni_kassel.vs.cn" id="1462373376006" name="CornerKick" comment="" masterPlan="false" utilityFunction="" utilityThreshold="0.1" destinationPath="" priority="0.0" minCardinality="0" maxCardinality="2147483647">
  <states id="1462373376007" name="Align" comment="" entryPoint="1462373376008"/>
  <states id="1462374515826" name="Align" comment="" entryPoint="1462373457908"/>
  <entryPoints id="1462373376008" name="ExecuteStandard" comment="" successRequired="false" minCardinality="0" maxCardinality="2147483647">
    <task>../../../../Misc/taskrepository.tsk#1439997010902</task>
    <state>#1462373376007</state>
  </entryPoints>
  <entryPoints id="1462373457908" name="ReceiveStandard" comment="" successRequired="false" minCardinality="0" maxCardinality="2147483647">
    <task>../../../../Misc/taskrepository.tsk#1439997023446</task>
    <state>#1462374515826</state>
  </entryPoints>
</alica:Plan>
