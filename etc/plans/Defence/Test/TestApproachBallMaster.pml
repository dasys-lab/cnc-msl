<?xml version="1.0" encoding="ASCII"?>
<alica:Plan xmi:version="2.0" xmlns:xmi="http://www.omg.org/XMI" xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance" xmlns:alica="http:///de.uni_kassel.vs.cn" id="1430324312981" name="TestApproachBallMaster" comment="" masterPlan="true" utilityFunction="" utilityThreshold="0.1" destinationPath="Plans/Defence/Test" priority="0.0" minCardinality="0" maxCardinality="2147483647">
  <states id="1430324312982" name="Stop" comment="" entryPoint="1430324312983">
    <plans xsi:type="alica:BehaviourConfiguration">../../GenericBehaviours/Stop.beh#1413992626194</plans>
    <inTransitions>#1431012024235</inTransitions>
    <inTransitions>#1431528885520</inTransitions>
    <inTransitions>#1431528887705</inTransitions>
    <inTransitions>#1431528896700</inTransitions>
    <inTransitions>#1431529013661</inTransitions>
    <outTransitions>#1430324473378</outTransitions>
    <outTransitions>#1431528624159</outTransitions>
    <outTransitions>#1431528884169</outTransitions>
  </states>
  <states id="1430324405240" name="AttackOpp" comment="">
    <plans xsi:type="alica:BehaviourConfiguration">../../Behaviours/AttackOpp.beh#1430324680117</plans>
    <inTransitions>#1430324473378</inTransitions>
    <outTransitions>#1431012024235</outTransitions>
  </states>
  <states id="1431528571070" name="DriveToStandingBall" comment="">
    <inTransitions>#1431528624159</inTransitions>
    <outTransitions>#1431528763807</outTransitions>
    <outTransitions>#1431528885520</outTransitions>
  </states>
  <states id="1431528632614" name="DriveToFieldline" comment="">
    <inTransitions>#1431528763807</inTransitions>
    <outTransitions>#1431528887705</outTransitions>
  </states>
  <states id="1431528769561" name="DriveToMovingBall" comment="">
    <inTransitions>#1431528884169</inTransitions>
    <outTransitions>#1431528896700</outTransitions>
    <outTransitions>#1431529011607</outTransitions>
  </states>
  <states id="1431528982294" name="DriveToMovFieldline" comment="">
    <inTransitions>#1431529011607</inTransitions>
    <outTransitions>#1431529013661</outTransitions>
  </states>
  <transitions id="1430324473378" name="MISSING_NAME" comment="Situation==Start" msg="">
    <preCondition id="1430324477939" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>#1430324312982</inState>
    <outState>#1430324405240</outState>
  </transitions>
  <transitions id="1431012024235" name="MISSING_NAME" comment="" msg="">
    <preCondition id="1431012032315" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>#1430324405240</inState>
    <outState>#1430324312982</outState>
  </transitions>
  <transitions id="1431528624159" name="MISSING_NAME" comment="" msg="">
    <preCondition id="1431528626141" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="false"/>
    <inState>#1430324312982</inState>
    <outState>#1431528571070</outState>
  </transitions>
  <transitions id="1431528763807" name="MISSING_NAME" comment="" msg="">
    <preCondition id="1431528765720" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>#1431528571070</inState>
    <outState>#1431528632614</outState>
  </transitions>
  <transitions id="1431528884169" name="MISSING_NAME" comment="" msg="">
    <preCondition id="1431528885384" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="false"/>
    <inState>#1430324312982</inState>
    <outState>#1431528769561</outState>
  </transitions>
  <transitions id="1431528885520" name="MISSING_NAME" comment="" msg="">
    <preCondition id="1431528887593" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="false"/>
    <inState>#1431528571070</inState>
    <outState>#1430324312982</outState>
  </transitions>
  <transitions id="1431528887705" name="MISSING_NAME" comment="" msg="">
    <preCondition id="1431528896437" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="false"/>
    <inState>#1431528632614</inState>
    <outState>#1430324312982</outState>
  </transitions>
  <transitions id="1431528896700" name="MISSING_NAME" comment="" msg="">
    <preCondition id="1431528902161" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="false"/>
    <inState>#1431528769561</inState>
    <outState>#1430324312982</outState>
  </transitions>
  <transitions id="1431529011607" name="MISSING_NAME" comment="" msg="">
    <preCondition id="1431529012684" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>#1431528769561</inState>
    <outState>#1431528982294</outState>
  </transitions>
  <transitions id="1431529013661" name="MISSING_NAME" comment="" msg="">
    <preCondition id="1431529014647" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="false"/>
    <inState>#1431528982294</inState>
    <outState>#1430324312982</outState>
  </transitions>
  <entryPoints id="1430324312983" name="MISSING_NAME" comment="" successRequired="false" minCardinality="0" maxCardinality="2147483647">
    <task>../../../Misc/taskrepository.tsk#1225112227903</task>
    <state>#1430324312982</state>
  </entryPoints>
</alica:Plan>
