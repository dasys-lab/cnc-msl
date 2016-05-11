<?xml version="1.0" encoding="ASCII"?>
<alica:Plan xmi:version="2.0" xmlns:xmi="http://www.omg.org/XMI" xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance" xmlns:alica="http:///de.uni_kassel.vs.cn" id="1458033644590" name="ReleaseOwnHalf" comment="" masterPlan="false" utilityFunction="" utilityThreshold="0.1" destinationPath="Plans/Defence" priority="0.0" minCardinality="0" maxCardinality="2147483647">
  <conditions xsi:type="alica:RuntimeCondition" id="1458033759784" name="NewRuntimeCondition" comment="" conditionString="" pluginName="DefaultPlugin">
    <vars>#1458033962376</vars>
    <vars>#1458033968280</vars>
  </conditions>
  <vars id="1458033962376" name="X" comment="" Type=""/>
  <vars id="1458033968280" name="Y" comment="" Type=""/>
  <states id="1458033660892" name="ReleaseOwnHalf" comment="" entryPoint="1458033660893">
    <parametrisation id="1458034180326" name="MISSING_NAME" comment="">
      <subplan xsi:type="alica:BehaviourConfiguration">../Behaviours/MoveToPointDynamic.beh#1458033795798</subplan>
      <subvar>../Behaviours/MoveToPointDynamic.beh#1458034153527</subvar>
      <var>#1458033962376</var>
    </parametrisation>
    <parametrisation id="1458034187225" name="MISSING_NAME" comment="">
      <subplan xsi:type="alica:BehaviourConfiguration">../Behaviours/MoveToPointDynamic.beh#1458033795798</subplan>
      <subvar>../Behaviours/MoveToPointDynamic.beh#1458034158628</subvar>
      <var>#1458033968280</var>
    </parametrisation>
    <plans xsi:type="alica:BehaviourConfiguration">../Behaviours/MoveToPointDynamic.beh#1458033795798</plans>
  </states>
  <entryPoints id="1458033660893" name="MISSING_NAME" comment="" successRequired="false" minCardinality="0" maxCardinality="2147483647">
    <task>../../Misc/taskrepository.tsk#1225112227903</task>
    <state>#1458033660892</state>
  </entryPoints>
</alica:Plan>
