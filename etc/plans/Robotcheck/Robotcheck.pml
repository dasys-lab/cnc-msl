<?xml version="1.0" encoding="ASCII"?>
<alica:Plan xmi:version="2.0" xmlns:xmi="http://www.omg.org/XMI" xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance" xmlns:alica="http:///de.uni_kassel.vs.cn" id="1456756058055" name="Robotcheck" comment="" destinationPath="Plans/Robotcheck" priority="0.0" minCardinality="0" maxCardinality="2147483647" masterPlan="false" utilityFunction="" utilityThreshold="0.1">
  <states id="1456756058056" name="RobotCheck" comment="">
    <plans xsi:type="alica:Behaviour">Robotcheck/RobotTest.beh#1456756113767</plans>
    <inTransitions>Robotcheck/Robotcheck.pml#1456841216933</inTransitions>
    <outTransitions>Robotcheck/Robotcheck.pml#1456841242506</outTransitions>
    <outTransitions>Robotcheck/Robotcheck.pml#1456841274321</outTransitions>
  </states>
  <states id="1456841196692" name="Stop" comment="" entryPoint="1456756058057">
    <inTransitions>Robotcheck/Robotcheck.pml#1456841242506</inTransitions>
    <outTransitions>Robotcheck/Robotcheck.pml#1456841216933</outTransitions>
  </states>
  <states xsi:type="alica:SuccessState" id="1456841265112" name="NewSuccessState" comment="">
    <inTransitions>Robotcheck/Robotcheck.pml#1456841274321</inTransitions>
  </states>
  <transitions id="1456841216933" name="MISSING_NAME" comment="Start" msg="">
    <preCondition id="1456841218640" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>Robotcheck/Robotcheck.pml#1456841196692</inState>
    <outState>Robotcheck/Robotcheck.pml#1456756058056</outState>
  </transitions>
  <transitions id="1456841242506" name="MISSING_NAME" comment="Stop" msg="">
    <preCondition id="1456841244750" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>Robotcheck/Robotcheck.pml#1456756058056</inState>
    <outState>Robotcheck/Robotcheck.pml#1456841196692</outState>
  </transitions>
  <transitions id="1456841274321" name="MISSING_NAME" comment="Success" msg="">
    <preCondition id="1456841276132" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>Robotcheck/Robotcheck.pml#1456756058056</inState>
    <outState>Robotcheck/Robotcheck.pml#1456841265112</outState>
  </transitions>
  <entryPoints id="1456756058057" name="MISSING_NAME" comment="" successRequired="false" minCardinality="0" maxCardinality="2147483647">
    <task>taskrepository.tsk#1225112227903</task>
    <state>Robotcheck/Robotcheck.pml#1456841196692</state>
  </entryPoints>
</alica:Plan>
