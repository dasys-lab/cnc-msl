<?xml version="1.0" encoding="ASCII"?>
<alica:Plan xmi:version="2.0" xmlns:xmi="http://www.omg.org/XMI" xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance" xmlns:alica="http:///de.uni_kassel.vs.cn" id="1426694865227" name="OppStandards" comment="" masterPlan="false" utilityFunction="" utilityThreshold="0.1" destinationPath="Plans/GameStrategy/OppStandards" priority="0.0" minCardinality="0" maxCardinality="2147483647">
  <states id="1426694865228" name="Dummy" comment="" entryPoint="1426694865229">
    <outTransitions>#1426697380342</outTransitions>
    <outTransitions>#1426697381777</outTransitions>
    <outTransitions>#1426697383087</outTransitions>
    <outTransitions>#1426697384201</outTransitions>
    <outTransitions>#1426697385447</outTransitions>
    <outTransitions>#1426697386854</outTransitions>
  </states>
  <states id="1426697288024" name="FreeKick" comment="">
    <plans xsi:type="alica:PlanType">OppFreeKick/OppFreeKickType.pty#1426698286970</plans>
    <inTransitions>#1426697383087</inTransitions>
    <outTransitions>#1426697394663</outTransitions>
  </states>
  <states id="1426697304719" name="GoalKick" comment="">
    <plans xsi:type="alica:PlanType">OppGoalKick/OppGoalKickType.pty#1426698237525</plans>
    <inTransitions>#1426697384201</inTransitions>
    <outTransitions>#1426697396242</outTransitions>
  </states>
  <states id="1426697310861" name="KickOff" comment="">
    <plans xsi:type="alica:PlanType">OppKickOff/OppKickOffType.pty#1426698358430</plans>
    <inTransitions>#1426697385447</inTransitions>
    <outTransitions>#1426697399357</outTransitions>
  </states>
  <states id="1426697315032" name="ThrowIn" comment="">
    <plans xsi:type="alica:PlanType">OppThrowIn/OppThrowInType.pty#1426698262931</plans>
    <inTransitions>#1426697386854</inTransitions>
    <outTransitions>#1426697401095</outTransitions>
  </states>
  <states id="1426697322419" name="Penalty" comment="">
    <plans xsi:type="alica:PlanType">OppPenalty/OppPenaltyType.pty#1426698389816</plans>
    <inTransitions>#1426697381777</inTransitions>
    <outTransitions>#1426697392346</outTransitions>
  </states>
  <states id="1426697332614" name="CornerKick" comment="">
    <plans xsi:type="alica:PlanType">OppCornerKick/OppCornerKickType.pty#1426698210110</plans>
    <inTransitions>#1426697380342</inTransitions>
    <outTransitions>#1426697388155</outTransitions>
  </states>
  <states xsi:type="alica:SuccessState" id="1426697367576" name="NewSuccessState" comment="">
    <inTransitions>#1426697388155</inTransitions>
    <inTransitions>#1426697392346</inTransitions>
    <inTransitions>#1426697394663</inTransitions>
    <inTransitions>#1426697396242</inTransitions>
    <inTransitions>#1426697399357</inTransitions>
    <inTransitions>#1426697401095</inTransitions>
  </states>
  <transitions id="1426697380342" name="MISSING_NAME" comment="Situation == OppCornerKick" msg="">
    <preCondition id="1426697381689" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>#1426694865228</inState>
    <outState>#1426697332614</outState>
  </transitions>
  <transitions id="1426697381777" name="MISSING_NAME" comment="Situation == OppPenalty" msg="">
    <preCondition id="1426697382983" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>#1426694865228</inState>
    <outState>#1426697322419</outState>
  </transitions>
  <transitions id="1426697383087" name="MISSING_NAME" comment="Situation == OppFreeKick" msg="">
    <preCondition id="1426697384088" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>#1426694865228</inState>
    <outState>#1426697288024</outState>
  </transitions>
  <transitions id="1426697384201" name="MISSING_NAME" comment="Situation == OppGoalKick" msg="">
    <preCondition id="1426697385319" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>#1426694865228</inState>
    <outState>#1426697304719</outState>
  </transitions>
  <transitions id="1426697385447" name="MISSING_NAME" comment="Situation == OppKickOff" msg="">
    <preCondition id="1426697386739" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>#1426694865228</inState>
    <outState>#1426697310861</outState>
  </transitions>
  <transitions id="1426697386854" name="MISSING_NAME" comment="Situation == OppTrowIn" msg="">
    <preCondition id="1426697388059" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>#1426694865228</inState>
    <outState>#1426697315032</outState>
  </transitions>
  <transitions id="1426697388155" name="MISSING_NAME" comment="anyChildSuccess" msg="">
    <preCondition id="1426697390236" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>#1426697332614</inState>
    <outState>#1426697367576</outState>
  </transitions>
  <transitions id="1426697392346" name="MISSING_NAME" comment="anyChildSuccess" msg="">
    <preCondition id="1426697394359" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>#1426697322419</inState>
    <outState>#1426697367576</outState>
  </transitions>
  <transitions id="1426697394663" name="MISSING_NAME" comment="anyChildSuccess" msg="">
    <preCondition id="1426697396107" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>#1426697288024</inState>
    <outState>#1426697367576</outState>
  </transitions>
  <transitions id="1426697396242" name="MISSING_NAME" comment="anyChildSuccess" msg="">
    <preCondition id="1426697399106" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>#1426697304719</inState>
    <outState>#1426697367576</outState>
  </transitions>
  <transitions id="1426697399357" name="MISSING_NAME" comment="anyChildSuccess" msg="">
    <preCondition id="1426697400936" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>#1426697310861</inState>
    <outState>#1426697367576</outState>
  </transitions>
  <transitions id="1426697401095" name="MISSING_NAME" comment="anyChildSuccess" msg="">
    <preCondition id="1426697403188" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>#1426697315032</inState>
    <outState>#1426697367576</outState>
  </transitions>
  <entryPoints id="1426694865229" name="MISSING_NAME" comment="" successRequired="true" minCardinality="0" maxCardinality="2147483647">
    <task>../../../Misc/taskrepository.tsk#1225112227903</task>
    <state>#1426694865228</state>
  </entryPoints>
</alica:Plan>
