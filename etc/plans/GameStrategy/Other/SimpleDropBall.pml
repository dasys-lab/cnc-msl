<?xml version="1.0" encoding="ASCII"?>
<alica:Plan xmi:version="2.0" xmlns:xmi="http://www.omg.org/XMI" xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance" xmlns:alica="http:///de.uni_kassel.vs.cn" id="1426696586622" name="SimpleDropBall" comment="" masterPlan="false" utilityFunction="" utilityThreshold="0.1" destinationPath="Plans/GameStrategy/Other" priority="0.0" minCardinality="0" maxCardinality="2147483647">
  <states id="1426696586623" name="Positioning" comment="" entryPoint="1426696586624">
    <plans xsi:type="alica:Plan">DropBallPositioning.pml#1455537014534</plans>
    <inTransitions>#1458562144787</inTransitions>
    <outTransitions>#1426696640520</outTransitions>
    <outTransitions>#1458562141333</outTransitions>
  </states>
  <states id="1426696626634" name="Execution" comment="">
    <plans xsi:type="alica:Plan">DropBallExecution.pml#1455537039421</plans>
    <inTransitions>#1426696640520</inTransitions>
    <outTransitions>#1426696641744</outTransitions>
    <outTransitions>#1458562143087</outTransitions>
  </states>
  <states xsi:type="alica:SuccessState" id="1426696638228" name="NewSuccessState" comment="">
    <inTransitions>#1426696641744</inTransitions>
  </states>
  <states id="1455538265766" name="DefendGoal" comment="" entryPoint="1455538209889">
    <plans xsi:type="alica:Plan">../../Goalie/Test/GoalieDefault.pml#1447254438614</plans>
  </states>
  <states id="1458562124636" name="Wander" comment="">
    <plans xsi:type="alica:Plan">WanderPlan.pml#1458553921358</plans>
    <inTransitions>#1458562141333</inTransitions>
    <inTransitions>#1458562143087</inTransitions>
    <outTransitions>#1458562144787</outTransitions>
  </states>
  <transitions id="1426696640520" name="MISSING_NAME" comment="Situation==start" msg="">
    <preCondition id="1426696641527" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>#1426696586623</inState>
    <outState>#1426696626634</outState>
  </transitions>
  <transitions id="1426696641744" name="MISSING_NAME" comment="anychildsuccess" msg="">
    <preCondition id="1426696642635" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>#1426696626634</inState>
    <outState>#1426696638228</outState>
  </transitions>
  <transitions id="1458562141333" name="MISSING_NAME" comment="EgoBall==nullptr" msg="">
    <preCondition id="1458562142910" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>#1426696586623</inState>
    <outState>#1458562124636</outState>
  </transitions>
  <transitions id="1458562143087" name="MISSING_NAME" comment="EgoBall==nullptr" msg="">
    <preCondition id="1458562144595" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>#1426696626634</inState>
    <outState>#1458562124636</outState>
  </transitions>
  <transitions id="1458562144787" name="MISSING_NAME" comment="EgoBall!=nullptr" msg="">
    <preCondition id="1458562147436" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>#1458562124636</inState>
    <outState>#1426696586623</outState>
  </transitions>
  <entryPoints id="1426696586624" name="MISSING_NAME" comment="" successRequired="true" minCardinality="0" maxCardinality="2147483647">
    <task>../../../Misc/taskrepository.tsk#1225112227903</task>
    <state>#1426696586623</state>
  </entryPoints>
  <entryPoints id="1455538209889" name="Keeper" comment="" successRequired="false" minCardinality="0" maxCardinality="1">
    <task>../../../Misc/taskrepository.tsk#1221754402444</task>
    <state>#1455538265766</state>
  </entryPoints>
</alica:Plan>
