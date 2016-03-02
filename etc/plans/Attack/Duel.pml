<?xml version="1.0" encoding="ASCII"?>
<alica:Plan xmi:version="2.0" xmlns:xmi="http://www.omg.org/XMI" xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance" xmlns:alica="http:///de.uni_kassel.vs.cn" id="1450178655416" name="Duel" comment="" masterPlan="false" utilityFunction="" utilityThreshold="0.1" destinationPath="Plans/Attack" priority="0.0" minCardinality="1" maxCardinality="2147483647">
  <states id="1454522133266" name="Wait" comment="" entryPoint="1454522099169">
    <plans xsi:type="alica:BehaviourConfiguration">../GenericBehaviours/Stop.beh#1413992626194</plans>
  </states>
  <states id="1455731653038" name="GetBallAggressively" comment="" entryPoint="1450178655418">
    <plans xsi:type="alica:BehaviourConfiguration">../Behaviours/GetBall.beh#1414840399972</plans>
    <outTransitions>#1455731694960</outTransitions>
  </states>
  <states id="1455731683998" name="Duel" comment="">
    <plans xsi:type="alica:BehaviourConfiguration">../Behaviours/Duel.beh#1450178707835</plans>
    <inTransitions>#1455731694960</inTransitions>
  </states>
  <transitions id="1455731694960" name="MISSING_NAME" comment="haveBall" msg="">
    <preCondition id="1455731695918" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>#1455731653038</inState>
    <outState>#1455731683998</outState>
  </transitions>
  <entryPoints id="1450178655418" name="MISSING_NAME" comment="" successRequired="false" minCardinality="1" maxCardinality="1">
    <task>../../Misc/taskrepository.tsk#1225112227903</task>
    <state>#1455731653038</state>
  </entryPoints>
  <entryPoints id="1454522099169" name="NewEntryPoint" comment="" successRequired="false" minCardinality="0" maxCardinality="2147483647">
    <task>../../Misc/taskrepository.tsk#1225112227903</task>
    <state>#1454522133266</state>
  </entryPoints>
</alica:Plan>
