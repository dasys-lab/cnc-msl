<?xml version="1.0" encoding="ASCII"?>
<alica:Plan xmi:version="2.0" xmlns:xmi="http://www.omg.org/XMI" xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance" xmlns:alica="http:///de.uni_kassel.vs.cn" id="1426697871183" name="GenericOppStandard" comment="" masterPlan="false" utilityFunction="" utilityThreshold="0.1" destinationPath="Plans/GameStrategy/OppStandards" priority="0.0" minCardinality="0" maxCardinality="2147483647">
  <states id="1426697871184" name="Positioning" comment="" entryPoint="1426697871185">
    <plans xsi:type="alica:Plan">GenericOppStandardPositioning.pml#1429108230432</plans>
    <outTransitions>#1426698467862</outTransitions>
  </states>
  <states id="1426698446747" name="Execution" comment="">
    <plans xsi:type="alica:Plan">GenericOppStandardExecuter.pml#1429109528736</plans>
    <inTransitions>#1426698467862</inTransitions>
    <outTransitions>#1426698472708</outTransitions>
  </states>
  <states xsi:type="alica:SuccessState" id="1426698470757" name="NewSuccessState" comment="">
    <inTransitions>#1426698472708</inTransitions>
  </states>
  <transitions id="1426698467862" name="MISSING_NAME" comment="Situation == Start" msg="">
    <preCondition id="1426698469169" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>#1426697871184</inState>
    <outState>#1426698446747</outState>
  </transitions>
  <transitions id="1426698472708" name="MISSING_NAME" comment="anyChildSuccess" msg="">
    <preCondition id="1426698473926" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>#1426698446747</inState>
    <outState>#1426698470757</outState>
  </transitions>
  <entryPoints id="1426697871185" name="MISSING_NAME" comment="" successRequired="true" minCardinality="0" maxCardinality="2147483647">
    <task>../../../Misc/taskrepository.tsk#1225112227903</task>
    <state>#1426697871184</state>
  </entryPoints>
</alica:Plan>
