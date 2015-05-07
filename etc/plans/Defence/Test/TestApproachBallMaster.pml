<?xml version="1.0" encoding="ASCII"?>
<alica:Plan xmi:version="2.0" xmlns:xmi="http://www.omg.org/XMI" xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance" xmlns:alica="http:///de.uni_kassel.vs.cn" id="1430324312981" name="TestApproachBallMaster" comment="" masterPlan="true" utilityFunction="" utilityThreshold="0.1" destinationPath="Plans/Defence/Test" priority="0.0" minCardinality="0" maxCardinality="2147483647">
  <states id="1430324312982" name="Stop" comment="" entryPoint="1430324312983">
    <plans xsi:type="alica:BehaviourConfiguration">../../GenericBehaviours/Stop.beh#1413992626194</plans>
    <outTransitions>#1430324473378</outTransitions>
  </states>
  <states id="1430324405240" name="AttackOpp" comment="">
    <plans xsi:type="alica:BehaviourConfiguration">../../Behaviours/AttackOpp.beh#1430324680117</plans>
    <inTransitions>#1430324473378</inTransitions>
  </states>
  <transitions id="1430324473378" name="MISSING_NAME" comment="Situation==Start" msg="">
    <preCondition id="1430324477939" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>#1430324312982</inState>
    <outState>#1430324405240</outState>
  </transitions>
  <entryPoints id="1430324312983" name="MISSING_NAME" comment="" successRequired="false" minCardinality="0" maxCardinality="2147483647">
    <task>../../../Misc/taskrepository.tsk#1225112227903</task>
    <state>#1430324312982</state>
  </entryPoints>
</alica:Plan>
