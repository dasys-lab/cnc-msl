<?xml version="1.0" encoding="ASCII"?>
<alica:Plan xmi:version="2.0" xmlns:xmi="http://www.omg.org/XMI" xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance" xmlns:alica="http:///de.uni_kassel.vs.cn" id="1434046634656" name="StandardAttack" comment="" masterPlan="false" utilityFunction="" utilityThreshold="0.1" destinationPath="" priority="0.0" minCardinality="0" maxCardinality="2147483647">
  <states id="1434046634657" name="GetBall" comment="" entryPoint="1434046634658">
    <inTransitions>#1434048722503</inTransitions>
    <inTransitions>#1434048731525</inTransitions>
    <outTransitions>#1434048720937</outTransitions>
    <outTransitions>#1434048723878</outTransitions>
  </states>
  <states id="1434048406725" name="Tackling" comment="">
    <inTransitions>#1434048723878</inTransitions>
    <inTransitions>#1434048729645</inTransitions>
    <outTransitions>#1434048731525</outTransitions>
    <outTransitions>#1434048734889</outTransitions>
  </states>
  <states id="1434048705508" name="HaveBall" comment="">
    <plans xsi:type="alica:Plan">Dribble.pml#1434049476066</plans>
    <inTransitions>#1434048720937</inTransitions>
    <inTransitions>#1434048734889</inTransitions>
    <outTransitions>#1434048722503</outTransitions>
    <outTransitions>#1434048729645</outTransitions>
  </states>
  <transitions id="1434048720937" name="MISSING_NAME" comment="" msg="">
    <preCondition id="1434048722207" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>#1434046634657</inState>
    <outState>#1434048705508</outState>
  </transitions>
  <transitions id="1434048722503" name="MISSING_NAME" comment="" msg="">
    <preCondition id="1434048723635" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>#1434048705508</inState>
    <outState>#1434046634657</outState>
  </transitions>
  <transitions id="1434048723878" name="MISSING_NAME" comment="" msg="">
    <preCondition id="1434048729350" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>#1434046634657</inState>
    <outState>#1434048406725</outState>
  </transitions>
  <transitions id="1434048729645" name="MISSING_NAME" comment="" msg="">
    <preCondition id="1434048731181" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>#1434048705508</inState>
    <outState>#1434048406725</outState>
  </transitions>
  <transitions id="1434048731525" name="MISSING_NAME" comment="" msg="">
    <preCondition id="1434048732966" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>#1434048406725</inState>
    <outState>#1434046634657</outState>
  </transitions>
  <transitions id="1434048734889" name="MISSING_NAME" comment="" msg="">
    <preCondition id="1434048737070" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>#1434048406725</inState>
    <outState>#1434048705508</outState>
  </transitions>
  <entryPoints id="1434046634658" name="Attack" comment="" successRequired="false" minCardinality="0" maxCardinality="2147483647">
    <task>../../Misc/taskrepository.tsk#1222613952469</task>
    <state>#1434046634657</state>
  </entryPoints>
</alica:Plan>
