<?xml version="1.0" encoding="ASCII"?>
<alica:Plan xmi:version="2.0" xmlns:xmi="http://www.omg.org/XMI" xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance" xmlns:alica="http:///de.uni_kassel.vs.cn" id="1458033644590" name="ReleaseOwnHalf" comment="" destinationPath="Plans/Defence" priority="0.0" minCardinality="0" maxCardinality="2147483647" masterPlan="false" utilityFunction="" utilityThreshold="0.1">
  <vars id="1458033962376" name="X" comment="" Type=""/>
  <vars id="1458033968280" name="Y" comment="" Type=""/>
  <runtimeCondition id="1458033759784" name="NewRuntimeCondition" comment="" conditionString="" pluginName="DefaultPlugin">
    <vars>Defence/ReleaseOwnHalf.pml#1458033962376</vars>
    <vars>Defence/ReleaseOwnHalf.pml#1458033968280</vars>
  </runtimeCondition>
  <states id="1458033660892" name="ReleaseOwnHalf" comment="" entryPoint="1458033660893">
    <parametrisation id="1458034180326" name="MISSING_NAME" comment="">
      <subplan xsi:type="alica:Behaviour">Behaviours/MoveToPointDynamic.beh#1456997073100</subplan>
      <subvar>Behaviours/MoveToPointDynamic.beh#1457000421766</subvar>
      <var>Defence/ReleaseOwnHalf.pml#1458033962376</var>
    </parametrisation>
    <parametrisation id="1458034187225" name="MISSING_NAME" comment="">
      <subplan xsi:type="alica:Behaviour">Behaviours/MoveToPointDynamic.beh#1456997073100</subplan>
      <subvar>Behaviours/MoveToPointDynamic.beh#1457000426918</subvar>
      <var>Defence/ReleaseOwnHalf.pml#1458033968280</var>
    </parametrisation>
    <plans xsi:type="alica:Behaviour">Behaviours/MoveToPointDynamic.beh#1456997073100</plans>
  </states>
  <entryPoints id="1458033660893" name="MISSING_NAME" comment="" successRequired="false" minCardinality="0" maxCardinality="2147483647">
    <task>taskrepository.tsk#1225112227903</task>
    <state>Defence/ReleaseOwnHalf.pml#1458033660892</state>
  </entryPoints>
</alica:Plan>
