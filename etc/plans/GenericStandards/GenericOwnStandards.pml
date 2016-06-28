<?xml version="1.0" encoding="ASCII"?>
<alica:Plan xmi:version="2.0" xmlns:xmi="http://www.omg.org/XMI" xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance" xmlns:alica="http:///de.uni_kassel.vs.cn" id="1430924951132" name="GenericOwnStandards" comment="" masterPlan="false" utilityFunction="" utilityThreshold="0.1" destinationPath="Plans/GenericStandards" priority="0.0" minCardinality="1" maxCardinality="2147483647">
  <states id="1430924951133" name="DummyOwnStd" comment="" entryPoint="1430925626532">
    <outTransitions>#1430925917001</outTransitions>
    <outTransitions>#1430925918585</outTransitions>
    <outTransitions>#1430925919866</outTransitions>
    <outTransitions>#1430925921331</outTransitions>
    <outTransitions>#1430925922843</outTransitions>
    <outTransitions>#1430925924151</outTransitions>
  </states>
  <states id="1430925718512" name="OwnThrowIn" comment="">
    <plans xsi:type="alica:PlanType">../Standards/Own/ThrowIn/ThrowIn.pty#1461672654276</plans>
    <inTransitions>#1430925917001</inTransitions>
    <inTransitions>#1458555879137</inTransitions>
    <outTransitions>#1430925960854</outTransitions>
    <outTransitions>#1458555221149</outTransitions>
  </states>
  <states id="1430925736746" name="OwnKickOff" comment="">
    <plans xsi:type="alica:PlanType">../Standards/Own/KickOff/OwnKickOff.pty#1438785270887</plans>
    <inTransitions>#1430925918585</inTransitions>
    <inTransitions>#1458555896457</inTransitions>
    <outTransitions>#1430925962882</outTransitions>
    <outTransitions>#1458555218949</outTransitions>
  </states>
  <states id="1430925743739" name="OwnGoalKick" comment="">
    <plans xsi:type="alica:PlanType">../Standards/Own/GoalKick/OwnGoalKick.pty#1465397323080</plans>
    <inTransitions>#1430925919866</inTransitions>
    <inTransitions>#1458555853125</inTransitions>
    <outTransitions>#1430925965662</outTransitions>
    <outTransitions>#1458555214716</outTransitions>
  </states>
  <states id="1430925751875" name="OwnFreeKick" comment="">
    <plans xsi:type="alica:PlanType">../Standards/Own/FreeKick/FreeKick.pty#1464783134585</plans>
    <inTransitions>#1430925922843</inTransitions>
    <inTransitions>#1458555819730</inTransitions>
    <outTransitions>#1430925971330</outTransitions>
    <outTransitions>#1458555205351</outTransitions>
  </states>
  <states id="1430925759928" name="OwnCornerKick" comment="">
    <plans xsi:type="alica:PlanType">../Standards/Own/Corner/OwnCorner.pty#1465397117125</plans>
    <inTransitions>#1430925924151</inTransitions>
    <inTransitions>#1458555788972</inTransitions>
    <outTransitions>#1430925973111</outTransitions>
    <outTransitions>#1458555207649</outTransitions>
  </states>
  <states id="1430925774870" name="OwnPenaltyKick" comment="">
    <plans xsi:type="alica:Plan">../Standards/Own/Penalty/InGame/OwnInGamePenalty.pml#1466936775181</plans>
    <inTransitions>#1430925921331</inTransitions>
    <outTransitions>#1430925967608</outTransitions>
  </states>
  <states xsi:type="alica:SuccessState" id="1430925945981" name="Success" comment="">
    <inTransitions>#1430925960854</inTransitions>
    <inTransitions>#1430925962882</inTransitions>
    <inTransitions>#1430925965662</inTransitions>
    <inTransitions>#1430925967608</inTransitions>
    <inTransitions>#1430925971330</inTransitions>
    <inTransitions>#1430925973111</inTransitions>
  </states>
  <states id="1431523938514" name="KeepGoal" comment="" entryPoint="1431523920444">
    <plans xsi:type="alica:Plan">../Goalie/Test/GoalieDefault.pml#1447254438614</plans>
  </states>
  <states id="1458555183582" name="WanderOwnStd" comment="">
    <plans xsi:type="alica:Plan">../GameStrategy/Other/WanderPlan.pml#1458553921358</plans>
    <inTransitions>#1458555205351</inTransitions>
    <inTransitions>#1458555207649</inTransitions>
    <inTransitions>#1458555214716</inTransitions>
    <inTransitions>#1458555218949</inTransitions>
    <inTransitions>#1458555221149</inTransitions>
    <outTransitions>#1458555788972</outTransitions>
    <outTransitions>#1458555819730</outTransitions>
    <outTransitions>#1458555853125</outTransitions>
    <outTransitions>#1458555879137</outTransitions>
    <outTransitions>#1458555896457</outTransitions>
  </states>
  <transitions id="1430925917001" name="MISSING_NAME" comment="situation = own throw in" msg="">
    <preCondition id="1430925918456" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>#1430924951133</inState>
    <outState>#1430925718512</outState>
  </transitions>
  <transitions id="1430925918585" name="MISSING_NAME" comment="situation = own kick off" msg="">
    <preCondition id="1430925919738" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>#1430924951133</inState>
    <outState>#1430925736746</outState>
  </transitions>
  <transitions id="1430925919866" name="MISSING_NAME" comment="situation = own goal kick" msg="">
    <preCondition id="1430925921267" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>#1430924951133</inState>
    <outState>#1430925743739</outState>
  </transitions>
  <transitions id="1430925921331" name="MISSING_NAME" comment="situation = own penalty kick" msg="">
    <preCondition id="1430925922740" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>#1430924951133</inState>
    <outState>#1430925774870</outState>
  </transitions>
  <transitions id="1430925922843" name="MISSING_NAME" comment="situation = own free kick" msg="">
    <preCondition id="1430925924055" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>#1430924951133</inState>
    <outState>#1430925751875</outState>
  </transitions>
  <transitions id="1430925924151" name="MISSING_NAME" comment="situation = own corner kick" msg="">
    <preCondition id="1430925925549" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>#1430924951133</inState>
    <outState>#1430925759928</outState>
  </transitions>
  <transitions id="1430925960854" name="trns" comment="any child task success  &amp;&amp; situation==start" msg="">
    <preCondition id="1430925962659" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>#1430925718512</inState>
    <outState>#1430925945981</outState>
  </transitions>
  <transitions id="1430925962882" name="MISSING_NAME" comment="any child success | any child failed" msg="">
    <preCondition id="1430925965501" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>#1430925736746</inState>
    <outState>#1430925945981</outState>
  </transitions>
  <transitions id="1430925965662" name="MISSING_NAME" comment="any child success &amp;&amp; situation==start" msg="">
    <preCondition id="1430925967520" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>#1430925743739</inState>
    <outState>#1430925945981</outState>
  </transitions>
  <transitions id="1430925967608" name="MISSING_NAME" comment="any child success &amp;&amp; situation==start" msg="">
    <preCondition id="1430925970967" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>#1430925774870</inState>
    <outState>#1430925945981</outState>
  </transitions>
  <transitions id="1430925971330" name="MISSING_NAME" comment="any child success &amp;&amp; situation==start" msg="">
    <preCondition id="1430925972838" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>#1430925751875</inState>
    <outState>#1430925945981</outState>
  </transitions>
  <transitions id="1430925973111" name="MISSING_NAME" comment="any child success &amp;&amp; situation==start" msg="">
    <preCondition id="1430925975558" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>#1430925759928</inState>
    <outState>#1430925945981</outState>
  </transitions>
  <transitions id="1458555205351" name="MISSING_NAME" comment="EgoBallPosition==nullptr;" msg="">
    <preCondition id="1458555207377" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>#1430925751875</inState>
    <outState>#1458555183582</outState>
  </transitions>
  <transitions id="1458555207649" name="MISSING_NAME" comment="EgoBallPosition==nullptr;" msg="">
    <preCondition id="1458555210443" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>#1430925759928</inState>
    <outState>#1458555183582</outState>
  </transitions>
  <transitions id="1458555214716" name="MISSING_NAME" comment="EgoBall=nullptr" msg="">
    <preCondition id="1458555218469" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>#1430925743739</inState>
    <outState>#1458555183582</outState>
  </transitions>
  <transitions id="1458555218949" name="MISSING_NAME" comment="EgoBallPosition==nullptr;" msg="">
    <preCondition id="1458555220837" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>#1430925736746</inState>
    <outState>#1458555183582</outState>
  </transitions>
  <transitions id="1458555221149" name="MISSING_NAME" comment="EgoBallPosition==nullptr;" msg="">
    <preCondition id="1458555223252" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>#1430925718512</inState>
    <outState>#1458555183582</outState>
  </transitions>
  <transitions id="1458555788972" name="MISSING_NAME" comment="EgoBallPosition!=nullptr &amp;&amp; Situation=OwnCorner;" msg="">
    <preCondition id="1458555791025" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>#1458555183582</inState>
    <outState>#1430925759928</outState>
  </transitions>
  <transitions id="1458555819730" name="MISSING_NAME" comment="EgoBallPosition!=nullptr &amp;&amp; Situation=OwnFreeKick;" msg="">
    <preCondition id="1458555821886" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>#1458555183582</inState>
    <outState>#1430925751875</outState>
  </transitions>
  <transitions id="1458555853125" name="MISSING_NAME" comment="EgoBallPosition!=nullptr &amp;&amp; Situation=OwnGoalK;" msg="">
    <preCondition id="1458555856023" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>#1458555183582</inState>
    <outState>#1430925743739</outState>
  </transitions>
  <transitions id="1458555879137" name="MISSING_NAME" comment="EgoBallPosition!=nullptr &amp;&amp; Situation=OwnThrowin;" msg="">
    <preCondition id="1458555881015" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>#1458555183582</inState>
    <outState>#1430925718512</outState>
  </transitions>
  <transitions id="1458555896457" name="MISSING_NAME" comment="EgoBallPosition!=nullptr &amp;&amp; Situation=OwnKickOff;" msg="">
    <preCondition id="1458555898277" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>#1458555183582</inState>
    <outState>#1430925736746</outState>
  </transitions>
  <entryPoints id="1430925626532" name="NewEntryPoint" comment="" successRequired="true" minCardinality="1" maxCardinality="2147483647">
    <task>../../Misc/taskrepository.tsk#1225112227903</task>
    <state>#1430924951133</state>
  </entryPoints>
  <entryPoints id="1431523920444" name="Keeper" comment="" successRequired="false" minCardinality="0" maxCardinality="1">
    <task>../../Misc/taskrepository.tsk#1221754402444</task>
    <state>#1431523938514</state>
  </entryPoints>
</alica:Plan>
