<?xml version="1.0" encoding="ASCII"?>
<alica:Plan xmi:version="2.0" xmlns:xmi="http://www.omg.org/XMI" xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance" xmlns:alica="http:///de.uni_kassel.vs.cn" id="1484139630501" name="DriveSquare" comment="" masterPlan="false" utilityFunction="" utilityThreshold="0.1" destinationPath="" priority="0.0" minCardinality="0" maxCardinality="2147483647">
  <states id="1484139630502" name="Stop" comment="" entryPoint="1484139630503">
    <plans xsi:type="alica:BehaviourConfiguration">../GenericBehaviours/Stop.beh#1413992626194</plans>
    <inTransitions>#1484143435427</inTransitions>
    <outTransitions>#1484143419403</outTransitions>
  </states>
  <states id="1484143351992" name="Up" comment="">
    <plans xsi:type="alica:BehaviourConfiguration">../GenericBehaviours/DriveToPoint.beh#1442921078802</plans>
    <inTransitions>#1484143419403</inTransitions>
    <outTransitions>#1484143424469</outTransitions>
  </states>
  <states id="1484143362945" name="Right" comment="">
    <plans xsi:type="alica:BehaviourConfiguration">../GenericBehaviours/DriveToPoint.beh#1484145108476</plans>
    <inTransitions>#1484143424469</inTransitions>
    <outTransitions>#1484143428833</outTransitions>
  </states>
  <states id="1484143370017" name="Down" comment="">
    <plans xsi:type="alica:BehaviourConfiguration">../GenericBehaviours/DriveToPoint.beh#1484145243859</plans>
    <inTransitions>#1484143428833</inTransitions>
    <outTransitions>#1484143433787</outTransitions>
  </states>
  <states id="1484143379874" name="Left" comment="">
    <plans xsi:type="alica:BehaviourConfiguration">../GenericBehaviours/DriveToPoint.beh#1484145325268</plans>
    <inTransitions>#1484143433787</inTransitions>
    <outTransitions>#1484143435427</outTransitions>
  </states>
  <transitions id="1484143419403" name="MISSING_NAME" comment="Drive up" msg="">
    <preCondition id="1484143421604" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>#1484139630502</inState>
    <outState>#1484143351992</outState>
  </transitions>
  <transitions id="1484143424469" name="MISSING_NAME" comment="Drive Right" msg="">
    <preCondition id="1484143426223" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>#1484143351992</inState>
    <outState>#1484143362945</outState>
  </transitions>
  <transitions id="1484143428833" name="MISSING_NAME" comment="Drive down" msg="">
    <preCondition id="1484143430778" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>#1484143362945</inState>
    <outState>#1484143370017</outState>
  </transitions>
  <transitions id="1484143433787" name="MISSING_NAME" comment="Drive left" msg="">
    <preCondition id="1484143435226" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>#1484143370017</inState>
    <outState>#1484143379874</outState>
  </transitions>
  <transitions id="1484143435427" name="MISSING_NAME" comment="Stop" msg="">
    <preCondition id="1484143436918" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>#1484143379874</inState>
    <outState>#1484139630502</outState>
  </transitions>
  <entryPoints id="1484139630503" name="MISSING_NAME" comment="" successRequired="false" minCardinality="0" maxCardinality="2147483647">
    <task>../../Misc/taskrepository.tsk#1225112227903</task>
    <state>#1484139630502</state>
  </entryPoints>
</alica:Plan>
