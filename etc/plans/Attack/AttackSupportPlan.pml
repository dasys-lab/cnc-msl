<?xml version="1.0" encoding="ASCII"?>
<alica:Plan xmi:version="2.0" xmlns:xmi="http://www.omg.org/XMI" xmlns:alica="http:///de.uni_kassel.vs.cn" id="1434046705214" name="AttackSupportPlan" comment="" masterPlan="false" utilityFunction="" utilityThreshold="0.1" destinationPath="Plans/Attack" priority="0.0" minCardinality="0" maxCardinality="2147483647">
  <states id="1434046705216" name="Midfield" comment="" entryPoint="1434046705217">
    <inTransitions>#1434110245599</inTransitions>
    <outTransitions>#1434110241637</outTransitions>
  </states>
  <states id="1434110211248" name="InterceptPass" comment="">
    <inTransitions>#1434110243412</inTransitions>
    <outTransitions>#1434110244556</outTransitions>
  </states>
  <states id="1434110212759" name="CatchPass" comment="">
    <inTransitions>#1434110241637</inTransitions>
    <inTransitions>#1434110244556</inTransitions>
    <outTransitions>#1434110243412</outTransitions>
    <outTransitions>#1434110245599</outTransitions>
  </states>
  <transitions id="1434110241637" name="MISSING_NAME" comment="is target for pass" msg="">
    <preCondition id="1434110243177" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>#1434046705216</inState>
    <outState>#1434110212759</outState>
  </transitions>
  <transitions id="1434110243412" name="MISSING_NAME" comment="ball conf high enough" msg="">
    <preCondition id="1434110244361" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>#1434110212759</inState>
    <outState>#1434110211248</outState>
  </transitions>
  <transitions id="1434110244556" name="MISSING_NAME" comment="pass msg empty" msg="">
    <preCondition id="1434110245418" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>#1434110211248</inState>
    <outState>#1434110212759</outState>
  </transitions>
  <transitions id="1434110245599" name="MISSING_NAME" comment="empty pass msg " msg="">
    <preCondition id="1434110246639" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>#1434110212759</inState>
    <outState>#1434046705216</outState>
  </transitions>
  <entryPoints id="1434046705217" name="AttackSupport" comment="" successRequired="false" minCardinality="0" maxCardinality="2147483647">
    <task>../../Misc/taskrepository.tsk#1225115536468</task>
    <state>#1434046705216</state>
  </entryPoints>
</alica:Plan>
