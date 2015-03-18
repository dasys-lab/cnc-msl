<?xml version="1.0" encoding="ASCII"?>
<alica:Plan xmi:version="2.0" xmlns:xmi="http://www.omg.org/XMI" xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance" xmlns:alica="http:///de.uni_kassel.vs.cn" id="1426697860439" name="GenericOwnStandard" comment="" masterPlan="false" utilityFunction="" utilityThreshold="0.1" destinationPath="" priority="0.0" minCardinality="0" maxCardinality="2147483647">
  <states id="1426697860440" name="Positioning" comment="" entryPoint="1426697860441">
    <outTransitions>#1426698518986</outTransitions>
  </states>
  <states id="1426698509635" name="Execution" comment="">
    <inTransitions>#1426698518986</inTransitions>
    <outTransitions>#1426698517666</outTransitions>
  </states>
  <states xsi:type="alica:SuccessState" id="1426698516012" name="NewSuccessState" comment="">
    <inTransitions>#1426698517666</inTransitions>
  </states>
  <transitions id="1426698517666" name="MISSING_NAME" comment="anyChildSuccess" msg="">
    <preCondition id="1426698518818" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>#1426698509635</inState>
    <outState>#1426698516012</outState>
  </transitions>
  <transitions id="1426698518986" name="MISSING_NAME" comment="Situation == Start" msg="">
    <preCondition id="1426698520600" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>#1426697860440</inState>
    <outState>#1426698509635</outState>
  </transitions>
  <entryPoints id="1426697860441" name="MISSING_NAME" comment="" successRequired="true" minCardinality="0" maxCardinality="2147483647">
    <task>../../../Misc/taskrepository.tsk#1225112227903</task>
    <state>#1426697860440</state>
  </entryPoints>
</alica:Plan>
