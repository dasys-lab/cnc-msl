<?xml version="1.0" encoding="ASCII"?>
<alica:Plan xmi:version="2.0" xmlns:xmi="http://www.omg.org/XMI" xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance" xmlns:alica="http:///de.uni_kassel.vs.cn" id="1441106995954" name="PassPlan" comment="" masterPlan="false" utilityFunction="" utilityThreshold="0.1" destinationPath="Plans/Attack/TestPlans" priority="0.0" minCardinality="3" maxCardinality="3">
  <states id="1441107009534" name="SearchForPassPoint" comment="" entryPoint="1441107009535">
    <plans xsi:type="alica:BehaviourConfiguration">../SearchForPassPoint.beh#1441107270872</plans>
    <inTransitions>#1441107214550</inTransitions>
    <outTransitions>#1441107212515</outTransitions>
  </states>
  <states id="1441107086220" name="WaitForPass" comment="" entryPoint="1441107038524">
    <plans xsi:type="alica:BehaviourConfiguration">../../GenericBehaviours/Stop.beh#1413992626194</plans>
  </states>
  <states id="1441107089085" name="WaitForPass" comment="" entryPoint="1441107040804">
    <plans xsi:type="alica:BehaviourConfiguration">../../GenericBehaviours/Stop.beh#1413992626194</plans>
  </states>
  <states id="1441107193775" name="Pass" comment="">
    <plans xsi:type="alica:BehaviourConfiguration">../AlignAndPassRapid.beh#1441108023281</plans>
    <inTransitions>#1441107212515</inTransitions>
    <outTransitions>#1441107214550</outTransitions>
  </states>
  <transitions id="1441107212515" name="MISSING_NAME" comment="situation freekick" msg="">
    <preCondition id="1441107213887" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>#1441107009534</inState>
    <outState>#1441107193775</outState>
  </transitions>
  <transitions id="1441107214550" name="MISSING_NAME" comment="situation goalkick" msg="">
    <preCondition id="1441107217606" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>#1441107193775</inState>
    <outState>#1441107009534</outState>
  </transitions>
  <entryPoints id="1441107009535" name="MISSING_NAME" comment="" successRequired="false" minCardinality="1" maxCardinality="1">
    <task>../../../Misc/taskrepository.tsk#1225112227903</task>
    <state>#1441107009534</state>
  </entryPoints>
  <entryPoints id="1441107038524" name="AttackSupport" comment="" successRequired="false" minCardinality="1" maxCardinality="1">
    <task>../../../Misc/taskrepository.tsk#1225115536468</task>
    <state>#1441107086220</state>
  </entryPoints>
  <entryPoints id="1441107040804" name="ReceivePassInGame" comment="" successRequired="false" minCardinality="1" maxCardinality="1">
    <task>../../../Misc/taskrepository.tsk#1307185798142</task>
    <state>#1441107089085</state>
  </entryPoints>
</alica:Plan>
