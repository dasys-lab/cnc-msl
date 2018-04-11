<?xml version="1.0" encoding="ASCII"?>
<alica:Plan xmi:version="2.0" xmlns:xmi="http://www.omg.org/XMI" xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance" xmlns:alica="http:///de.uni_kassel.vs.cn" id="1523356831385" name="TestThrowInPassAlignment" comment="" masterPlan="false" utilityFunction="" utilityThreshold="0.1" destinationPath="" priority="0.0" minCardinality="0" maxCardinality="2147483647">
  <states id="1523356831386" name="AlignExec" comment="" entryPoint="1523356831387">
    <outTransitions>#1523357599178</outTransitions>
  </states>
  <states id="1523356936733" name="AlignRec" comment="" entryPoint="1523356924027"/>
  <states id="1523356978219" name="AlignAlternativeRec" comment="" entryPoint="1523356926500">
    <plans xsi:type="alica:BehaviourConfiguration">../../ThrowIn/PositionAlternativeReceiver.beh#1462978671719</plans>
  </states>
  <states id="1523357593225" name="Pass" comment="">
    <inTransitions>#1523357599178</inTransitions>
  </states>
  <transitions id="1523357599178" name="MISSING_NAME" comment="" msg="">
    <preCondition id="1523357600273" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>#1523356831386</inState>
    <outState>#1523357593225</outState>
  </transitions>
  <entryPoints id="1523356831387" name="ExecuteStandard" comment="" successRequired="false" minCardinality="0" maxCardinality="2147483647">
    <task>../../../../../Misc/taskrepository.tsk#1439997010902</task>
    <state>#1523356831386</state>
  </entryPoints>
  <entryPoints id="1523356924027" name="ReceiveStandard" comment="" successRequired="false" minCardinality="0" maxCardinality="2147483647">
    <task>../../../../../Misc/taskrepository.tsk#1439997023446</task>
    <state>#1523356936733</state>
  </entryPoints>
  <entryPoints id="1523356926500" name="AlternativeReceive" comment="" successRequired="false" minCardinality="0" maxCardinality="2147483647">
    <task>../../../../../Misc/taskrepository.tsk#1462360858945</task>
    <state>#1523356978219</state>
  </entryPoints>
</alica:Plan>
