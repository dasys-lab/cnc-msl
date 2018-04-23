<?xml version="1.0" encoding="ASCII"?>
<alica:Plan xmi:version="2.0" xmlns:xmi="http://www.omg.org/XMI" xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance" xmlns:alica="http:///de.uni_kassel.vs.cn" id="1521280800018" name="DriveToOppsiteSite" comment="" masterPlan="false" utilityFunction="" utilityThreshold="0.1" destinationPath="Plans/TestPlans/DribbleTestMOS" priority="0.0" minCardinality="0" maxCardinality="2147483647">
  <states id="1521280800020" name="InitalPositioning" comment="" entryPoint="1521280800021">
    <plans xsi:type="alica:BehaviourConfiguration">../../GenericBehaviours/DriveAndAlignToPoint.beh#1521281255214</plans>
    <outTransitions>#1521280930325</outTransitions>
  </states>
  <states id="1521280896157" name="WaitForBall" comment="">
    <inTransitions>#1521280930325</inTransitions>
    <outTransitions>#1521280986276</outTransitions>
  </states>
  <states id="1521280936955" name="DriveToOtherSite" comment="">
    <plans xsi:type="alica:BehaviourConfiguration">../../GenericBehaviours/DriveAndAlignToPoint.beh#1521281418695</plans>
    <inTransitions>#1521280993055</inTransitions>
    <outTransitions>#1521281058637</outTransitions>
  </states>
  <states id="1521280974028" name="WaitShortly" comment="">
    <inTransitions>#1521280986276</inTransitions>
    <outTransitions>#1521280993055</outTransitions>
  </states>
  <states xsi:type="alica:SuccessState" id="1521281050940" name="NewSuccessState" comment="">
    <inTransitions>#1521281058637</inTransitions>
  </states>
  <transitions id="1521280930325" name="MISSING_NAME" comment="" msg="">
    <preCondition id="1521280931730" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>#1521280800020</inState>
    <outState>#1521280896157</outState>
  </transitions>
  <transitions id="1521280986276" name="MISSING_NAME" comment="" msg="">
    <preCondition id="1521280987477" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>#1521280896157</inState>
    <outState>#1521280974028</outState>
  </transitions>
  <transitions id="1521280993055" name="MISSING_NAME" comment="" msg="">
    <preCondition id="1521280994181" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>#1521280974028</inState>
    <outState>#1521280936955</outState>
  </transitions>
  <transitions id="1521281058637" name="MISSING_NAME" comment="" msg="">
    <preCondition id="1521281060067" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>#1521280936955</inState>
    <outState>#1521281050940</outState>
  </transitions>
  <entryPoints id="1521280800021" name="MISSING_NAME" comment="" successRequired="true" minCardinality="0" maxCardinality="2147483647">
    <task>../../../Misc/taskrepository.tsk#1225112227903</task>
    <state>#1521280800020</state>
  </entryPoints>
</alica:Plan>
