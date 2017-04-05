<?xml version="1.0" encoding="ASCII"?>
<alica:Plan xmi:version="2.0" xmlns:xmi="http://www.omg.org/XMI" xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance" xmlns:alica="http:///de.uni_kassel.vs.cn" id="1436268896671" name="PassPlay" comment="" destinationPath="Plans/Attack" priority="0.0" minCardinality="0" maxCardinality="2147483647" masterPlan="false" utilityFunction="" utilityThreshold="0.1">
  <states id="1436268896672" name="Dribble" comment="" entryPoint="1436268896674">
    <plans xsi:type="alica:Behaviour">Attack/SearchForPassPoint.beh#1436269017402</plans>
    <plans xsi:type="alica:Behaviour">Behaviours/ShovelSelect.beh#1434199834892</plans>
    <plans xsi:type="alica:Plan">Attack/Dribble.pml#1434049476066</plans>
    <plans xsi:type="alica:Behaviour">Dribble/DribbleControl.beh#1449742071382</plans>
    <inTransitions>Attack/PassPlay.pml#1436268944412</inTransitions>
    <outTransitions>Attack/PassPlay.pml#1436268942088</outTransitions>
  </states>
  <states id="1436268931449" name="Pass" comment="">
    <plans xsi:type="alica:Behaviour">Attack/AlignAndPassRapid.beh#1436269063295</plans>
    <plans xsi:type="alica:Behaviour">Behaviours/ShovelSelect.beh#1434199834892</plans>
    <plans xsi:type="alica:Behaviour">Dribble/DribbleControl.beh#1449742071382</plans>
    <inTransitions>Attack/PassPlay.pml#1436268942088</inTransitions>
    <outTransitions>Attack/PassPlay.pml#1436268944412</outTransitions>
  </states>
  <transitions id="1436268942088" name="MISSING_NAME" comment="Success" msg="">
    <preCondition id="1436268944209" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>Attack/PassPlay.pml#1436268896672</inState>
    <outState>Attack/PassPlay.pml#1436268931449</outState>
  </transitions>
  <transitions id="1436268944412" name="MISSING_NAME" comment="pass fails" msg="">
    <preCondition id="1436268945305" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>Attack/PassPlay.pml#1436268931449</inState>
    <outState>Attack/PassPlay.pml#1436268896672</outState>
  </transitions>
  <entryPoints id="1436268896674" name="MISSING_NAME" comment="" successRequired="false" minCardinality="0" maxCardinality="2147483647">
    <task>taskrepository.tsk#1225112227903</task>
    <state>Attack/PassPlay.pml#1436268896672</state>
  </entryPoints>
</alica:Plan>
