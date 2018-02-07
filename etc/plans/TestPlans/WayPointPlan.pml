<?xml version="1.0" encoding="ASCII"?>
<alica:Plan xmi:version="2.0" xmlns:xmi="http://www.omg.org/XMI" xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance" xmlns:alica="http:///de.uni_kassel.vs.cn" id="1513182751590" name="WayPointPlan" comment="" masterPlan="false" utilityFunction="" utilityThreshold="0.1" destinationPath="Plans/TestPlans" priority="0.0" minCardinality="0" maxCardinality="2147483647">
  <states id="1513182781377" name="Wait" comment="" entryPoint="1513182781378">
    <plans xsi:type="alica:BehaviourConfiguration">../GenericBehaviours/Stop.beh#1413992626194</plans>
    <inTransitions>#1513182971945</inTransitions>
    <inTransitions>#1513182975102</inTransitions>
    <inTransitions>#1513182977357</inTransitions>
    <inTransitions>#1513182979433</inTransitions>
    <inTransitions>#1513182981534</inTransitions>
    <outTransitions>#1513182954370</outTransitions>
    <outTransitions>#1513182956007</outTransitions>
    <outTransitions>#1513182958003</outTransitions>
    <outTransitions>#1513182960034</outTransitions>
    <outTransitions>#1513182968804</outTransitions>
  </states>
  <states id="1513182861309" name="SoftSquare" comment="">
    <plans xsi:type="alica:Plan">SoftSquarePlan.pml#1513183613607</plans>
    <inTransitions>#1513182954370</inTransitions>
    <outTransitions>#1513182981534</outTransitions>
  </states>
  <states id="1513182863558" name="NewState" comment="">
    <plans xsi:type="alica:Plan">Circle.pml#1517049779097</plans>
    <inTransitions>#1513182956007</inTransitions>
    <outTransitions>#1513182979433</outTransitions>
  </states>
  <states id="1513182865455" name="ZigZag-looking" comment="">
    <plans xsi:type="alica:Plan">ZigZag-looking.pml#1517415924695</plans>
    <inTransitions>#1513182968804</inTransitions>
    <outTransitions>#1513182971945</outTransitions>
  </states>
  <states id="1513182867816" name="NewState" comment="">
    <plans xsi:type="alica:Plan">curves.pml#1517056420902</plans>
    <inTransitions>#1513182960034</inTransitions>
    <outTransitions>#1513182975102</outTransitions>
  </states>
  <states id="1513182870017" name="ZigZag-small" comment="">
    <plans xsi:type="alica:Plan">DribbleControlTest/ZigZag-small.pml#1517050419596</plans>
    <inTransitions>#1513182958003</inTransitions>
    <outTransitions>#1513182977357</outTransitions>
  </states>
  <transitions id="1513182954370" name="MISSING_NAME" comment="Opp Kick Off" msg="">
    <preCondition id="1513182955846" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>#1513182781377</inState>
    <outState>#1513182861309</outState>
  </transitions>
  <transitions id="1513182956007" name="MISSING_NAME" comment="Opp Free Kick" msg="">
    <preCondition id="1513182957882" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>#1513182781377</inState>
    <outState>#1513182863558</outState>
  </transitions>
  <transitions id="1513182958003" name="MISSING_NAME" comment="Opp Goal Kick" msg="">
    <preCondition id="1513182959777" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>#1513182781377</inState>
    <outState>#1513182870017</outState>
  </transitions>
  <transitions id="1513182960034" name="MISSING_NAME" comment="Opp Throwin" msg="">
    <preCondition id="1513182968659" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>#1513182781377</inState>
    <outState>#1513182867816</outState>
  </transitions>
  <transitions id="1513182968804" name="MISSING_NAME" comment="Opp Corner" msg="">
    <preCondition id="1513182971720" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>#1513182781377</inState>
    <outState>#1513182865455</outState>
  </transitions>
  <transitions id="1513182971945" name="MISSING_NAME" comment="success" msg="">
    <preCondition id="1513182974957" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>#1513182865455</inState>
    <outState>#1513182781377</outState>
  </transitions>
  <transitions id="1513182975102" name="MISSING_NAME" comment="success" msg="">
    <preCondition id="1513182977188" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>#1513182867816</inState>
    <outState>#1513182781377</outState>
  </transitions>
  <transitions id="1513182977357" name="MISSING_NAME" comment="success" msg="">
    <preCondition id="1513182979288" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>#1513182870017</inState>
    <outState>#1513182781377</outState>
  </transitions>
  <transitions id="1513182979433" name="MISSING_NAME" comment="success" msg="">
    <preCondition id="1513182981413" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>#1513182863558</inState>
    <outState>#1513182781377</outState>
  </transitions>
  <transitions id="1513182981534" name="MISSING_NAME" comment="success" msg="">
    <preCondition id="1513182984479" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>#1513182861309</inState>
    <outState>#1513182781377</outState>
  </transitions>
  <entryPoints id="1513182781378" name="MISSING_NAME" comment="" successRequired="false" minCardinality="0" maxCardinality="2147483647">
    <task>../../Misc/taskrepository.tsk#1225112227903</task>
    <state>#1513182781377</state>
  </entryPoints>
</alica:Plan>
