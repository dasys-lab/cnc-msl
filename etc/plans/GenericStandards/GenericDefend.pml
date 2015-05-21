<?xml version="1.0" encoding="ASCII"?>
<alica:Plan xmi:version="2.0" xmlns:xmi="http://www.omg.org/XMI" xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance" xmlns:alica="http:///de.uni_kassel.vs.cn" id="1432133473779" name="GenericDefend" comment="" masterPlan="false" utilityFunction="" utilityThreshold="0.1" destinationPath="Plans/GenericStandards" priority="0.0" minCardinality="0" maxCardinality="2147483647">
  <states id="1432133473780" name="Block" comment="" entryPoint="1432209646142">
    <outTransitions>#1432134616763</outTransitions>
  </states>
  <states id="1432134414694" name="Defend" comment="" entryPoint="1432209644156">
    <outTransitions>#1432134614588</outTransitions>
  </states>
  <states xsi:type="alica:SuccessState" id="1432134598332" name="Success" comment="GetBall">
    <inTransitions>#1432134614588</inTransitions>
    <inTransitions>#1432134616763</inTransitions>
  </states>
  <transitions id="1432134614588" name="MISSING_NAME" comment="GetBall" msg="">
    <preCondition id="1432134616532" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>#1432134414694</inState>
    <outState>#1432134598332</outState>
  </transitions>
  <transitions id="1432134616763" name="MISSING_NAME" comment="" msg="">
    <preCondition id="1432134618673" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>#1432133473780</inState>
    <outState>#1432134598332</outState>
  </transitions>
  <entryPoints id="1432209644156" name="Defend" comment="" successRequired="false" minCardinality="0" maxCardinality="2147483647">
    <task>../../Misc/taskrepository.tsk#1225115406909</task>
    <state>#1432134414694</state>
  </entryPoints>
  <entryPoints id="1432209646142" name="Blocker" comment="" successRequired="false" minCardinality="0" maxCardinality="2147483647">
    <task>../../Misc/taskrepository.tsk#1432209050494</task>
    <state>#1432133473780</state>
  </entryPoints>
</alica:Plan>
