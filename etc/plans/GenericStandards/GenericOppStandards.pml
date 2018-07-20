<?xml version="1.0" encoding="ASCII"?>
<alica:Plan xmi:version="2.0" xmlns:xmi="http://www.omg.org/XMI" xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance" xmlns:alica="http:///de.uni_kassel.vs.cn" id="1432132075122" name="GenericOppStandards" comment="" masterPlan="false" utilityFunction="" utilityThreshold="0.1" destinationPath="Plans/GenericStandards" priority="0.0" minCardinality="0" maxCardinality="2147483647">
  <states id="1432132075123" name="Dummy" comment="" entryPoint="1432209594646">
    <outTransitions>#1432132365582</outTransitions>
    <outTransitions>#1432132368065</outTransitions>
    <outTransitions>#1432132369863</outTransitions>
    <outTransitions>#1432132371135</outTransitions>
    <outTransitions>#1432132372448</outTransitions>
    <outTransitions>#1432132373553</outTransitions>
  </states>
  <states id="1432132173301" name="KeepGoal" comment="" entryPoint="1432209615271">
    <plans xsi:type="alica:Plan">../Goalie/Test/GoalieDefault.pml#1447254438614</plans>
  </states>
  <states id="1432132225844" name="OppThrowIn" comment="">
    <plans xsi:type="alica:PlanType">../Standards/Opponent/FreeKick/OppFreeKick.pty#1447870848925</plans>
    <inTransitions>#1432132365582</inTransitions>
    <outTransitions>#1432132429602</outTransitions>
  </states>
  <states id="1432132283325" name="OppKickOff" comment="">
    <plans xsi:type="alica:PlanType">../Standards/Opponent/FreeKick/OppFreeKick.pty#1447870848925</plans>
    <inTransitions>#1432132368065</inTransitions>
    <outTransitions>#1432132432185</outTransitions>
  </states>
  <states id="1432132299937" name="OppGoalKick" comment="">
    <plans xsi:type="alica:PlanType">../Standards/Opponent/FreeKick/OppFreeKick.pty#1447870848925</plans>
    <inTransitions>#1432132369863</inTransitions>
    <outTransitions>#1432132434174</outTransitions>
  </states>
  <states id="1432132312429" name="OppPenaltyKick" comment="">
    <plans xsi:type="alica:Plan">../Standards/Opponent/Penalty/OppInGamePenalty.pml#1466968232004</plans>
    <inTransitions>#1432132371135</inTransitions>
    <outTransitions>#1432132436499</outTransitions>
  </states>
  <states id="1432132325758" name="OppFreeKick" comment="">
    <plans xsi:type="alica:PlanType">../Standards/Opponent/FreeKick/OppFreeKick.pty#1447870848925</plans>
    <inTransitions>#1432132373553</inTransitions>
    <outTransitions>#1432132441185</outTransitions>
  </states>
  <states id="1432132336430" name="OppCornerKick" comment="">
    <plans xsi:type="alica:PlanType">../Standards/Opponent/FreeKick/OppFreeKick.pty#1447870848925</plans>
    <inTransitions>#1432132372448</inTransitions>
    <outTransitions>#1432132438388</outTransitions>
  </states>
  <states xsi:type="alica:SuccessState" id="1432132398157" name="Success" comment="">
    <inTransitions>#1432132429602</inTransitions>
    <inTransitions>#1432132432185</inTransitions>
    <inTransitions>#1432132434174</inTransitions>
    <inTransitions>#1432132436499</inTransitions>
    <inTransitions>#1432132438388</inTransitions>
    <inTransitions>#1432132441185</inTransitions>
  </states>
  <transitions id="1432132365582" name="MISSING_NAME" comment="Sitaution= OppThrowin" msg="">
    <preCondition id="1432132367859" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>#1432132075123</inState>
    <outState>#1432132225844</outState>
  </transitions>
  <transitions id="1432132368065" name="MISSING_NAME" comment="Sitaution= OppKickOff" msg="">
    <preCondition id="1432132369717" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>#1432132075123</inState>
    <outState>#1432132283325</outState>
  </transitions>
  <transitions id="1432132369863" name="MISSING_NAME" comment="Sitaution= OppGoalKick" msg="">
    <preCondition id="1432132371055" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>#1432132075123</inState>
    <outState>#1432132299937</outState>
  </transitions>
  <transitions id="1432132371135" name="MISSING_NAME" comment="Situation= OppPenaltyKick" msg="">
    <preCondition id="1432132372365" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>#1432132075123</inState>
    <outState>#1432132312429</outState>
  </transitions>
  <transitions id="1432132372448" name="MISSING_NAME" comment="Sitaution= OppCornerKick" msg="">
    <preCondition id="1432132373471" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>#1432132075123</inState>
    <outState>#1432132336430</outState>
  </transitions>
  <transitions id="1432132373553" name="MISSING_NAME" comment="Sitaution= OppFreeKick" msg="">
    <preCondition id="1432132374502" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>#1432132075123</inState>
    <outState>#1432132325758</outState>
  </transitions>
  <transitions id="1432132429602" name="MISSING_NAME" comment="any child success" msg="">
    <preCondition id="1432132432036" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>#1432132225844</inState>
    <outState>#1432132398157</outState>
  </transitions>
  <transitions id="1432132432185" name="MISSING_NAME" comment="any child success" msg="">
    <preCondition id="1432132434051" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>#1432132283325</inState>
    <outState>#1432132398157</outState>
  </transitions>
  <transitions id="1432132434174" name="MISSING_NAME" comment="any child success" msg="">
    <preCondition id="1432132436308" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>#1432132299937</inState>
    <outState>#1432132398157</outState>
  </transitions>
  <transitions id="1432132436499" name="MISSING_NAME" comment="any child success" msg="">
    <preCondition id="1432132438181" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>#1432132312429</inState>
    <outState>#1432132398157</outState>
  </transitions>
  <transitions id="1432132438388" name="MISSING_NAME" comment="any child success" msg="">
    <preCondition id="1432132441034" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>#1432132336430</inState>
    <outState>#1432132398157</outState>
  </transitions>
  <transitions id="1432132441185" name="MISSING_NAME" comment="any child success" msg="">
    <preCondition id="1432132442910" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>#1432132325758</inState>
    <outState>#1432132398157</outState>
  </transitions>
  <entryPoints id="1432209594646" name="DefaultTask" comment="" successRequired="true" minCardinality="0" maxCardinality="2147483647">
    <task>../../Misc/taskrepository.tsk#1225112227903</task>
    <state>#1432132075123</state>
  </entryPoints>
  <entryPoints id="1432209615271" name="Keeper" comment="" successRequired="false" minCardinality="0" maxCardinality="2147483647">
    <task>../../Misc/taskrepository.tsk#1221754402444</task>
    <state>#1432132173301</state>
  </entryPoints>
</alica:Plan>
