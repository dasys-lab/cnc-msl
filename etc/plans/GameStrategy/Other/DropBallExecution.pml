<?xml version="1.0" encoding="ASCII"?>
<alica:Plan xmi:version="2.0" xmlns:xmi="http://www.omg.org/XMI" xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance" xmlns:alica="http:///de.uni_kassel.vs.cn" id="1455537039421" name="DropBallExecution" comment="" masterPlan="false" utilityFunction="" utilityThreshold="0.1" destinationPath="Plans/GameStrategy/Other" priority="0.0" minCardinality="0" maxCardinality="2147483647">
  <states id="1455537039422" name="DoSomethingCool" comment="" entryPoint="1455537039423">
    <plans xsi:type="alica:BehaviourConfiguration">../../GenericBehaviours/Stop.beh#1413992626194</plans>
    <outTransitions>#1455537182943</outTransitions>
  </states>
  <states xsi:type="alica:SuccessState" id="1455537179349" name="NewSuccessState" comment="">
    <inTransitions>#1455537182943</inTransitions>
  </states>
  <transitions id="1455537182943" name="MISSING_NAME" comment="true" msg="">
    <preCondition id="1455537184881" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>#1455537039422</inState>
    <outState>#1455537179349</outState>
  </transitions>
  <entryPoints id="1455537039423" name="MISSING_NAME" comment="" successRequired="false" minCardinality="0" maxCardinality="2147483647">
    <task>../../../Misc/taskrepository.tsk#1225112227903</task>
    <state>#1455537039422</state>
  </entryPoints>
</alica:Plan>
