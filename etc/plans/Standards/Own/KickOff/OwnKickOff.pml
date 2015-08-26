<?xml version="1.0" encoding="ASCII"?>
<alica:Plan xmi:version="2.0" xmlns:xmi="http://www.omg.org/XMI" xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance" xmlns:alica="http:///de.uni_kassel.vs.cn" id="1438785376159" name="OwnKickOff" comment="" masterPlan="false" utilityFunction="" utilityThreshold="0.1" destinationPath="Plans/Standards/Own/KickOff" priority="0.0" minCardinality="0" maxCardinality="2147483647">
  <states id="1438785376160" name="PosExec" comment="" entryPoint="1438785376161">
    <plans xsi:type="alica:BehaviourConfiguration">../../../Behaviours/PositionExecutor.beh#1438790487994</plans>
    <outTransitions>#1440600382572</outTransitions>
  </states>
  <states id="1438785501838" name="PosRecv" comment="" entryPoint="1438785392109">
    <plans xsi:type="alica:BehaviourConfiguration">../../../Behaviours/PositionReceiver.beh#1439379352605</plans>
  </states>
  <states id="1440600350522" name="AlignExec" comment="">
    <plans xsi:type="alica:BehaviourConfiguration">../../../Behaviours/AlignExecutor.beh#1440600507552</plans>
    <inTransitions>#1440600382572</inTransitions>
  </states>
  <transitions id="1440600382572" name="MISSING_NAME" comment="when on position" msg="">
    <preCondition id="1440600384398" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>#1438785376160</inState>
    <outState>#1440600350522</outState>
  </transitions>
  <entryPoints id="1438785376161" name="MISSING_NAME" comment="" successRequired="false" minCardinality="0" maxCardinality="2147483647">
    <task>../../../../Misc/taskrepository.tsk#1439997010902</task>
    <state>#1438785376160</state>
  </entryPoints>
  <entryPoints id="1438785392109" name="MISSING_NAME" comment="" successRequired="false" minCardinality="0" maxCardinality="2147483647">
    <task>../../../../Misc/taskrepository.tsk#1439997023446</task>
    <state>#1438785501838</state>
  </entryPoints>
</alica:Plan>
