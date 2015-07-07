<?xml version="1.0" encoding="ASCII"?>
<alica:Plan xmi:version="2.0" xmlns:xmi="http://www.omg.org/XMI" xmlns:alica="http:///de.uni_kassel.vs.cn" id="1434115664325" name="RunFree" comment="" masterPlan="false" utilityFunction="" utilityThreshold="0.1" destinationPath="Plans/Attack" priority="0.0" minCardinality="0" maxCardinality="2147483647">
  <states id="1434115664326" name="Release" comment="" entryPoint="1434115664327">
    <inTransitions>#1434115869697</inTransitions>
    <outTransitions>#1434115867679</outTransitions>
  </states>
  <states id="1434115849689" name="CatchPass" comment="">
    <inTransitions>#1434115867679</inTransitions>
    <inTransitions>#1434115872185</inTransitions>
    <outTransitions>#1434115869697</outTransitions>
    <outTransitions>#1434115870604</outTransitions>
  </states>
  <states id="1434115852118" name="InterceptPass" comment="">
    <inTransitions>#1434115870604</inTransitions>
    <outTransitions>#1434115872185</outTransitions>
  </states>
  <transitions id="1434115867679" name="MISSING_NAME" comment="is target for pass" msg="">
    <preCondition id="1434115869504" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>#1434115664326</inState>
    <outState>#1434115849689</outState>
  </transitions>
  <transitions id="1434115869697" name="MISSING_NAME" comment="empty pass msg " msg="">
    <preCondition id="1434115870439" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>#1434115849689</inState>
    <outState>#1434115664326</outState>
  </transitions>
  <transitions id="1434115870604" name="MISSING_NAME" comment="ball conf high enough" msg="">
    <preCondition id="1434115871872" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>#1434115849689</inState>
    <outState>#1434115852118</outState>
  </transitions>
  <transitions id="1434115872185" name="MISSING_NAME" comment="pass msg empty" msg="">
    <preCondition id="1434115873056" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>#1434115852118</inState>
    <outState>#1434115849689</outState>
  </transitions>
  <entryPoints id="1434115664327" name="InGamePassReceiver" comment="" successRequired="false" minCardinality="0" maxCardinality="2147483647">
    <task>../../Misc/taskrepository.tsk#1307185798142</task>
    <state>#1434115664326</state>
  </entryPoints>
</alica:Plan>
