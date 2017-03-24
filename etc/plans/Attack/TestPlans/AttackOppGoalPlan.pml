<?xml version="1.0" encoding="ASCII"?>
<alica:Plan xmi:version="2.0" xmlns:xmi="http://www.omg.org/XMI" xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance" xmlns:alica="http:///de.uni_kassel.vs.cn" id="1437902649389" name="AttackOppGoalPlan" comment="" masterPlan="false" utilityFunction="" utilityThreshold="0.1" destinationPath="Plans/Attack/TestPlans" priority="0.0" minCardinality="0" maxCardinality="2147483647">
  <states id="1437902649390" name="GetBall" comment="" entryPoint="1437902649391">
    <plans xsi:type="alica:Behaviour">Behaviours/GetBall.beh#1414840399972</plans>
    <inTransitions>#1437903089522</inTransitions>
    <outTransitions>#1437903088529</outTransitions>
  </states>
  <states id="1437902693350" name="DribbleToOppPenaltySpot" comment="">
    <plans xsi:type="alica:Behaviour">Attack/DribbleToAttackPoint.beh#1436855860607</plans>
    <plans xsi:type="alica:Behaviour">Behaviours/ShovelSelect.beh#1435156714286</plans>
    <inTransitions>#1437903090929</inTransitions>
    <inTransitions>#1437903097451</inTransitions>
    <outTransitions>#1437903092957</outTransitions>
    <outTransitions>#1437903093831</outTransitions>
  </states>
  <states id="1437902704803" name="DribbleToOwnPenaltySpot" comment="">
    <plans xsi:type="alica:Behaviour">Attack/DribbleToAttackPoint.beh#1437391438054</plans>
    <plans xsi:type="alica:Behaviour">Behaviours/ShovelSelect.beh#1435156714286</plans>
    <inTransitions>#1437903088529</inTransitions>
    <outTransitions>#1437903089522</outTransitions>
    <outTransitions>#1437903097451</outTransitions>
  </states>
  <states xsi:type="alica:SuccessState" id="1437902754765" name="NewSuccessState" comment="">
    <inTransitions>#1437903095217</inTransitions>
  </states>
  <states id="1437902762624" name="Shoot" comment="">
    <plans xsi:type="alica:Behaviour">Behaviours/GoalKick.beh#1415205578139</plans>
    <plans xsi:type="alica:Behaviour">Behaviours/ShovelSelect.beh#1435156714286</plans>
    <inTransitions>#1437903093831</inTransitions>
    <outTransitions>#1437903095217</outTransitions>
  </states>
  <states id="1437903060557" name="GetBall" comment="">
    <plans xsi:type="alica:Behaviour">Behaviours/GetBall.beh#1414840399972</plans>
    <inTransitions>#1437903092957</inTransitions>
    <outTransitions>#1437903090929</outTransitions>
  </states>
  <transitions id="1437903088529" name="MISSING_NAME" comment="haveBall" msg="">
    <preCondition id="1437903089358" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>#1437902649390</inState>
    <outState>#1437902704803</outState>
  </transitions>
  <transitions id="1437903089522" name="MISSING_NAME" comment="lostBall" msg="">
    <preCondition id="1437903090726" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>#1437902704803</inState>
    <outState>#1437902649390</outState>
  </transitions>
  <transitions id="1437903090929" name="MISSING_NAME" comment="haveBall" msg="">
    <preCondition id="1437903092576" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>#1437903060557</inState>
    <outState>#1437902693350</outState>
  </transitions>
  <transitions id="1437903092957" name="MISSING_NAME" comment="lostBall" msg="">
    <preCondition id="1437903093659" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>#1437902693350</inState>
    <outState>#1437903060557</outState>
  </transitions>
  <transitions id="1437903093831" name="MISSING_NAME" comment="anyChildSuccess" msg="">
    <preCondition id="1437903095031" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>#1437902693350</inState>
    <outState>#1437902762624</outState>
  </transitions>
  <transitions id="1437903095217" name="MISSING_NAME" comment="anyChildSuccess" msg="">
    <preCondition id="1437903097184" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>#1437902762624</inState>
    <outState>#1437902754765</outState>
  </transitions>
  <transitions id="1437903097451" name="MISSING_NAME" comment="anyChildSuccess" msg="">
    <preCondition id="1437903100138" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>#1437902704803</inState>
    <outState>#1437902693350</outState>
  </transitions>
  <entryPoints id="1437902649391" name="MISSING_NAME" comment="" successRequired="true" minCardinality="0" maxCardinality="2147483647">
    <task>taskrepository.tsk#1225112227903</task>
    <state>#1437902649390</state>
  </entryPoints>
</alica:Plan>
