<?xml version="1.0" encoding="ASCII"?>
<alica:Plan xmi:version="2.0" xmlns:xmi="http://www.omg.org/XMI" xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance" xmlns:alica="http:///de.uni_kassel.vs.cn" id="1426694875113" name="OwnStandards" comment="" masterPlan="false" utilityFunction="" utilityThreshold="0.1" destinationPath="Plans/GameStrategy/OwnStandards" priority="0.0" minCardinality="0" maxCardinality="2147483647">
  <states id="1426694875114" name="Dummy" comment="" entryPoint="1426694875115">
    <outTransitions>#1426696896987</outTransitions>
    <outTransitions>#1426696898114</outTransitions>
    <outTransitions>#1426696899284</outTransitions>
    <outTransitions>#1426696900543</outTransitions>
    <outTransitions>#1426696902014</outTransitions>
    <outTransitions>#1426696903255</outTransitions>
  </states>
  <states id="1426696810907" name="FreeKick" comment="">
    <plans xsi:type="alica:PlanType">OwnFreeKick/OwnFreeKickType.pty#1426698015369</plans>
    <inTransitions>#1426696898114</inTransitions>
    <outTransitions>#1426696954517</outTransitions>
  </states>
  <states id="1426696817258" name="GoalKick" comment="">
    <plans xsi:type="alica:PlanType">OwnGoalKick/OwnGoalKickType.pty#1426698043458</plans>
    <inTransitions>#1426696899284</inTransitions>
    <outTransitions>#1426696956228</outTransitions>
  </states>
  <states id="1426696823465" name="Penalty" comment="">
    <plans xsi:type="alica:PlanType">OwnPenalty/OwnPenaltyType.pty#1426698098722</plans>
    <inTransitions>#1426696902014</inTransitions>
    <outTransitions>#1426696957920</outTransitions>
  </states>
  <states id="1426696855134" name="ThrowIn" comment="">
    <plans xsi:type="alica:PlanType">OwnThrowIn/OwnThrowInType.pty#1426698121099</plans>
    <inTransitions>#1426696900543</inTransitions>
    <outTransitions>#1426696961286</outTransitions>
  </states>
  <states id="1426696862884" name="CornerKick" comment="">
    <plans xsi:type="alica:PlanType">OwnCornerKick/OwnCornerKickType.pty#1426697882552</plans>
    <inTransitions>#1426696896987</inTransitions>
    <outTransitions>#1426696952905</outTransitions>
  </states>
  <states id="1426696872622" name="KickOff" comment="">
    <plans xsi:type="alica:PlanType">OwnKickOff/OwnKickOffType.pty#1426698076341</plans>
    <inTransitions>#1426696903255</inTransitions>
    <outTransitions>#1426696959405</outTransitions>
  </states>
  <states xsi:type="alica:SuccessState" id="1426696949478" name="NewSuccessState" comment="">
    <inTransitions>#1426696952905</inTransitions>
    <inTransitions>#1426696954517</inTransitions>
    <inTransitions>#1426696956228</inTransitions>
    <inTransitions>#1426696957920</inTransitions>
    <inTransitions>#1426696959405</inTransitions>
    <inTransitions>#1426696961286</inTransitions>
  </states>
  <transitions id="1426696896987" name="MISSING_NAME" comment="Situation == CornerKick" msg="">
    <preCondition id="1426696897969" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>#1426694875114</inState>
    <outState>#1426696862884</outState>
  </transitions>
  <transitions id="1426696898114" name="MISSING_NAME" comment="Situation == FreeKick" msg="">
    <preCondition id="1426696899156" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>#1426694875114</inState>
    <outState>#1426696810907</outState>
  </transitions>
  <transitions id="1426696899284" name="MISSING_NAME" comment="Situation == GoalKick" msg="">
    <preCondition id="1426696900390" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>#1426694875114</inState>
    <outState>#1426696817258</outState>
  </transitions>
  <transitions id="1426696900543" name="MISSING_NAME" comment="Situation == ThrowIn" msg="">
    <preCondition id="1426696901894" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>#1426694875114</inState>
    <outState>#1426696855134</outState>
  </transitions>
  <transitions id="1426696902014" name="MISSING_NAME" comment="Situation == Penalty" msg="">
    <preCondition id="1426696903119" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>#1426694875114</inState>
    <outState>#1426696823465</outState>
  </transitions>
  <transitions id="1426696903255" name="MISSING_NAME" comment="Situation == KickOff" msg="">
    <preCondition id="1426696904473" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>#1426694875114</inState>
    <outState>#1426696872622</outState>
  </transitions>
  <transitions id="1426696952905" name="MISSING_NAME" comment="anyChildSuccess" msg="">
    <preCondition id="1426696954307" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>#1426696862884</inState>
    <outState>#1426696949478</outState>
  </transitions>
  <transitions id="1426696954517" name="MISSING_NAME" comment="anyChildSuccess" msg="">
    <preCondition id="1426696956076" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>#1426696810907</inState>
    <outState>#1426696949478</outState>
  </transitions>
  <transitions id="1426696956228" name="MISSING_NAME" comment="anyChildSuccess" msg="">
    <preCondition id="1426696957736" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>#1426696817258</inState>
    <outState>#1426696949478</outState>
  </transitions>
  <transitions id="1426696957920" name="MISSING_NAME" comment="anyChildSuccess" msg="">
    <preCondition id="1426696959204" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>#1426696823465</inState>
    <outState>#1426696949478</outState>
  </transitions>
  <transitions id="1426696959405" name="MISSING_NAME" comment="anyChildSuccess" msg="">
    <preCondition id="1426696961046" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>#1426696872622</inState>
    <outState>#1426696949478</outState>
  </transitions>
  <transitions id="1426696961286" name="MISSING_NAME" comment="anyChildSuccess" msg="">
    <preCondition id="1426696963628" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>#1426696855134</inState>
    <outState>#1426696949478</outState>
  </transitions>
  <entryPoints id="1426694875115" name="MISSING_NAME" comment="" successRequired="true" minCardinality="0" maxCardinality="2147483647">
    <task>../../../Misc/taskrepository.tsk#1225112227903</task>
    <state>#1426694875114</state>
  </entryPoints>
</alica:Plan>
