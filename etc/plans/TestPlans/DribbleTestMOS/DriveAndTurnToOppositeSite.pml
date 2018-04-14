<?xml version="1.0" encoding="ASCII"?>
<alica:Plan xmi:version="2.0" xmlns:xmi="http://www.omg.org/XMI" xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance" xmlns:alica="http:///de.uni_kassel.vs.cn" id="1521283132783" name="DriveAndTurnToOppositeSite" comment="" masterPlan="false" utilityFunction="" utilityThreshold="0.1" destinationPath="Plans/TestPlans/DribbleTestMOS" priority="0.0" minCardinality="0" maxCardinality="2147483647">
  <states id="1521283132784" name="InitialPos" comment="" entryPoint="1521283132785">
    <plans xsi:type="alica:BehaviourConfiguration">../../GenericBehaviours/DriveAndAlignToPoint.beh#1523718438212</plans>
    <outTransitions>#1521283235652</outTransitions>
  </states>
  <states id="1521283171226" name="WaitShortly" comment="">
    <inTransitions>#1521283236778</inTransitions>
    <outTransitions>#1521283238079</outTransitions>
  </states>
  <states id="1521283172827" name="DriveToSite" comment="">
    <plans xsi:type="alica:BehaviourConfiguration">../../GenericBehaviours/DriveAndAlignToPoint.beh#1523718464367</plans>
    <inTransitions>#1521283238079</inTransitions>
    <outTransitions>#1521283248282</outTransitions>
  </states>
  <states id="1521283174412" name="WaitForBall" comment="">
    <inTransitions>#1521283235652</inTransitions>
    <outTransitions>#1521283236778</outTransitions>
  </states>
  <states xsi:type="alica:SuccessState" id="1521283242241" name="NewSuccessState" comment="">
    <inTransitions>#1521283248282</inTransitions>
  </states>
  <transitions id="1521283235652" name="MISSING_NAME" comment="" msg="">
    <preCondition id="1521283236610" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>#1521283132784</inState>
    <outState>#1521283174412</outState>
  </transitions>
  <transitions id="1521283236778" name="MISSING_NAME" comment="" msg="">
    <preCondition id="1521283237926" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>#1521283174412</inState>
    <outState>#1521283171226</outState>
  </transitions>
  <transitions id="1521283238079" name="MISSING_NAME" comment="" msg="">
    <preCondition id="1521283239559" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>#1521283171226</inState>
    <outState>#1521283172827</outState>
  </transitions>
  <transitions id="1521283248282" name="MISSING_NAME" comment="" msg="">
    <preCondition id="1521283249550" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>#1521283172827</inState>
    <outState>#1521283242241</outState>
  </transitions>
  <entryPoints id="1521283132785" name="MISSING_NAME" comment="" successRequired="true" minCardinality="0" maxCardinality="2147483647">
    <task>../../../Misc/taskrepository.tsk#1225112227903</task>
    <state>#1521283132784</state>
  </entryPoints>
</alica:Plan>
