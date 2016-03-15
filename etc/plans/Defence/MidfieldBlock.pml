<?xml version="1.0" encoding="ASCII"?>
<alica:Plan xmi:version="2.0" xmlns:xmi="http://www.omg.org/XMI" xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance" xmlns:alica="http:///de.uni_kassel.vs.cn" id="1458033620834" name="MidfieldBlock" comment="" masterPlan="false" utilityFunction="" utilityThreshold="0.1" destinationPath="Plans/Defence" priority="0.0" minCardinality="0" maxCardinality="2147483647">
  <conditions xsi:type="alica:RuntimeCondition" id="1458033723845" name="NewRuntimeCondition" comment="" conditionString="" pluginName="DefaultPlugin">
    <vars>#1458034250626</vars>
    <vars>#1458034254267</vars>
  </conditions>
  <vars id="1458034250626" name="X" comment="" Type=""/>
  <vars id="1458034254267" name="Y" comment="" Type=""/>
  <states id="1458033636228" name="MidfieldBlock" comment="" entryPoint="1458033636229">
    <parametrisation id="1458034333856" name="MISSING_NAME" comment="">
      <subplan xsi:type="alica:BehaviourConfiguration">OneGernericInGameBlocker.beh#1458034300406</subplan>
      <subvar>OneGernericInGameBlocker.beh#1458034324013</subvar>
      <var>#1458034254267</var>
    </parametrisation>
    <parametrisation id="1458034337885" name="MISSING_NAME" comment="">
      <subplan xsi:type="alica:BehaviourConfiguration">OneGernericInGameBlocker.beh#1458034300406</subplan>
      <subvar>OneGernericInGameBlocker.beh#1458034318907</subvar>
      <var>#1458034250626</var>
    </parametrisation>
    <plans xsi:type="alica:BehaviourConfiguration">OneGernericInGameBlocker.beh#1458034300406</plans>
    <outTransitions>#1458033704233</outTransitions>
  </states>
  <states xsi:type="alica:SuccessState" id="1458033701713" name="BlockingFinished" comment="">
    <inTransitions>#1458033704233</inTransitions>
  </states>
  <transitions id="1458033704233" name="MISSING_NAME" comment="" msg="">
    <preCondition id="1458033705136" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>#1458033636228</inState>
    <outState>#1458033701713</outState>
  </transitions>
  <entryPoints id="1458033636229" name="MISSING_NAME" comment="" successRequired="true" minCardinality="0" maxCardinality="2147483647">
    <task>../../Misc/taskrepository.tsk#1225112227903</task>
    <state>#1458033636228</state>
  </entryPoints>
</alica:Plan>
