<?xml version="1.0" encoding="ASCII"?>
<alica:Plan xmi:version="2.0" xmlns:xmi="http://www.omg.org/XMI" xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance" xmlns:alica="http:///de.uni_kassel.vs.cn" id="1513175912125" name="TestLongPassMaster" comment="" masterPlan="true" utilityFunction="" utilityThreshold="0.1" destinationPath="Plans/Standards/Own/Test" priority="0.0" minCardinality="0" maxCardinality="2147483647">
  <states id="1513175912126" name="Stop" comment="" entryPoint="1513175912127">
    <inTransitions>#1513175932789</inTransitions>
    <outTransitions>#1513175931943</outTransitions>
  </states>
  <states id="1513175928013" name="Test" comment="">
    <plans xsi:type="alica:Plan">LongPassBasement.pml#1513176006534</plans>
    <inTransitions>#1513175931943</inTransitions>
    <outTransitions>#1513175932789</outTransitions>
  </states>
  <transitions id="1513175931943" name="MISSING_NAME" comment="situation start" msg="">
    <preCondition id="1513175932613" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>#1513175912126</inState>
    <outState>#1513175928013</outState>
  </transitions>
  <transitions id="1513175932789" name="MISSING_NAME" comment="situation stop or success" msg="">
    <preCondition id="1513175933840" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>#1513175928013</inState>
    <outState>#1513175912126</outState>
  </transitions>
  <entryPoints id="1513175912127" name="MISSING_NAME" comment="" successRequired="false" minCardinality="0" maxCardinality="2147483647">
    <task>../../../../Misc/taskrepository.tsk#1225112227903</task>
    <state>#1513175912126</state>
  </entryPoints>
</alica:Plan>
