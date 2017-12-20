<?xml version="1.0" encoding="ASCII"?>
<alica:Plan xmi:version="2.0" xmlns:xmi="http://www.omg.org/XMI" xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance" xmlns:alica="http:///de.uni_kassel.vs.cn" id="1513176006534" name="LongPassBasement" comment="pm for own id" masterPlan="false" utilityFunction="" utilityThreshold="0.1" destinationPath="Plans/Standards/Own/Test" priority="0.0" minCardinality="2" maxCardinality="2">
  <states id="1513176006535" name="AlignExec" comment="" entryPoint="1513176006536">
    <plans xsi:type="alica:BehaviourConfiguration">Align4PassTest.beh#1513609404882</plans>
    <plans xsi:type="alica:BehaviourConfiguration">../../../Behaviours/ShovelSelect.beh#1435156714286</plans>
    <outTransitions>#1513176124567</outTransitions>
  </states>
  <states id="1513176035432" name="AlignRec" comment="" entryPoint="1513176019047">
    <plans xsi:type="alica:BehaviourConfiguration">Align4PassTest.beh#1513784309003</plans>
    <plans xsi:type="alica:BehaviourConfiguration">../../../Behaviours/ShovelSelect.beh#1435156811453</plans>
    <outTransitions>#1513176174538</outTransitions>
  </states>
  <states id="1513176098858" name="GrabBall" comment="">
    <plans xsi:type="alica:BehaviourConfiguration">../../../GenericStandards/StandardAlignAndGrab.beh#1513176284031</plans>
    <plans xsi:type="alica:BehaviourConfiguration">../../../Behaviours/ShovelSelect.beh#1435156714286</plans>
    <inTransitions>#1513176124567</inTransitions>
    <inTransitions>#1513176153574</inTransitions>
    <outTransitions>#1513176125489</outTransitions>
  </states>
  <states id="1513176103139" name="Pass" comment="">
    <plans xsi:type="alica:BehaviourConfiguration">../ThrowIn/ThrowInPass.beh#1462363309950</plans>
    <plans xsi:type="alica:BehaviourConfiguration">../../../Behaviours/ShovelSelect.beh#1435156714286</plans>
    <inTransitions>#1513176125489</inTransitions>
    <outTransitions>#1513176126396</outTransitions>
    <outTransitions>#1513176153574</outTransitions>
  </states>
  <states id="1513176106132" name="Stop" comment="">
    <plans xsi:type="alica:BehaviourConfiguration">../../../GenericBehaviours/Stop.beh#1413992626194</plans>
    <plans xsi:type="alica:BehaviourConfiguration">../../../Behaviours/ShovelSelect.beh#1435156714286</plans>
    <inTransitions>#1513176126396</inTransitions>
  </states>
  <states xsi:type="alica:SuccessState" id="1513176112686" name="Success" comment="">
    <inTransitions>#1513176175579</inTransitions>
  </states>
  <states id="1513176169159" name="Receive" comment="">
    <plans xsi:type="alica:BehaviourConfiguration">../../../GenericBehaviours/InterceptCarefully.beh#1427703234654</plans>
    <plans xsi:type="alica:BehaviourConfiguration">../../../Behaviours/ShovelSelect.beh#1435156811453</plans>
    <inTransitions>#1513786331557</inTransitions>
    <outTransitions>#1513176175579</outTransitions>
    <outTransitions>#1513786448936</outTransitions>
  </states>
  <states id="1513786312435" name="CatchPass" comment="">
    <plans xsi:type="alica:BehaviourConfiguration">../../../Attack/CatchPass.beh#1440754543898</plans>
    <plans xsi:type="alica:BehaviourConfiguration">../../../Behaviours/ShovelSelect.beh#1435156811453</plans>
    <inTransitions>#1513176174538</inTransitions>
    <inTransitions>#1513786448936</inTransitions>
    <outTransitions>#1513786331557</outTransitions>
  </states>
  <transitions id="1513176124567" name="MISSING_NAME" comment="situation start" msg="">
    <preCondition id="1513176125409" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>#1513176006535</inState>
    <outState>#1513176098858</outState>
  </transitions>
  <transitions id="1513176125489" name="MISSING_NAME" comment="any child success" msg="">
    <preCondition id="1513176126275" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>#1513176098858</inState>
    <outState>#1513176103139</outState>
  </transitions>
  <transitions id="1513176126396" name="MISSING_NAME" comment="passed" msg="">
    <preCondition id="1513176126957" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>#1513176103139</inState>
    <outState>#1513176106132</outState>
  </transitions>
  <transitions id="1513176153574" name="MISSING_NAME" comment="lostball" msg="">
    <preCondition id="1513176155095" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>#1513176103139</inState>
    <outState>#1513176098858</outState>
  </transitions>
  <transitions id="1513176174538" name="MISSING_NAME" comment="pm for own id" msg="">
    <preCondition id="1513176175329" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>#1513176035432</inState>
    <outState>#1513786312435</outState>
  </transitions>
  <transitions id="1513176175579" name="MISSING_NAME" comment="haveball or ball out of field" msg="">
    <preCondition id="1513176176397" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>#1513176169159</inState>
    <outState>#1513176112686</outState>
  </transitions>
  <transitions id="1513786331557" name="MISSING_NAME" comment="close to ball" msg="">
    <preCondition id="1513786332716" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>#1513786312435</inState>
    <outState>#1513176169159</outState>
  </transitions>
  <transitions id="1513786448936" name="MISSING_NAME" comment="ball far away" msg="">
    <preCondition id="1513786449862" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>#1513176169159</inState>
    <outState>#1513786312435</outState>
  </transitions>
  <entryPoints id="1513176006536" name="ExecuteStandard" comment="" successRequired="false" minCardinality="1" maxCardinality="1">
    <task>../../../../Misc/taskrepository.tsk#1439997010902</task>
    <state>#1513176006535</state>
  </entryPoints>
  <entryPoints id="1513176019047" name="ReceiveStandard" comment="" successRequired="true" minCardinality="1" maxCardinality="1">
    <task>../../../../Misc/taskrepository.tsk#1439997023446</task>
    <state>#1513176035432</state>
  </entryPoints>
</alica:Plan>
