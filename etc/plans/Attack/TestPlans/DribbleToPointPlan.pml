<?xml version="1.0" encoding="ASCII"?>
<alica:Plan xmi:version="2.0" xmlns:xmi="http://www.omg.org/XMI" xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance" xmlns:alica="http:///de.uni_kassel.vs.cn" id="1436960829485" name="DribbleToPointPlan" comment="" destinationPath="Plans/Attack/TestPlans" priority="0.0" minCardinality="0" maxCardinality="2147483647" masterPlan="false" utilityFunction="" utilityThreshold="0.1">
  <states id="1436960962060" name="DribbleToOppPenaltySpot" comment="">
    <plans xsi:type="alica:Behaviour">Attack/DribbleToAttackPoint.beh#1436855838589</plans>
    <inTransitions>Attack/TestPlans/DribbleToPointPlan.pml#1437390978582</inTransitions>
    <inTransitions>Attack/TestPlans/DribbleToPointPlan.pml#1437483408716</inTransitions>
    <outTransitions>Attack/TestPlans/DribbleToPointPlan.pml#1437390979982</outTransitions>
    <outTransitions>Attack/TestPlans/DribbleToPointPlan.pml#1437483410283</outTransitions>
  </states>
  <states id="1437390908773" name="DribbleToOwnPenaltySpot" comment="" entryPoint="1436960854733">
    <plans xsi:type="alica:Behaviour">Attack/DribbleToAttackPoint.beh#1436855838589</plans>
    <inTransitions>Attack/TestPlans/DribbleToPointPlan.pml#1437390979982</inTransitions>
    <inTransitions>Attack/TestPlans/DribbleToPointPlan.pml#1437483382709</inTransitions>
    <outTransitions>Attack/TestPlans/DribbleToPointPlan.pml#1437390978582</outTransitions>
    <outTransitions>Attack/TestPlans/DribbleToPointPlan.pml#1437483384370</outTransitions>
  </states>
  <states id="1437483357988" name="GrabBall" comment="">
    <plans xsi:type="alica:Behaviour">Behaviours/AttackOpp.beh#1430324527403</plans>
    <inTransitions>Attack/TestPlans/DribbleToPointPlan.pml#1437483384370</inTransitions>
    <outTransitions>Attack/TestPlans/DribbleToPointPlan.pml#1437483382709</outTransitions>
  </states>
  <states id="1437483395818" name="GrabBall" comment="">
    <plans xsi:type="alica:Behaviour">Behaviours/AttackOpp.beh#1430324527403</plans>
    <inTransitions>Attack/TestPlans/DribbleToPointPlan.pml#1437483410283</inTransitions>
    <outTransitions>Attack/TestPlans/DribbleToPointPlan.pml#1437483408716</outTransitions>
  </states>
  <transitions id="1437390978582" name="MISSING_NAME" comment="anyChildSuccess &amp;&amp;  situation own penalty" msg="">
    <preCondition id="1437390979726" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>Attack/TestPlans/DribbleToPointPlan.pml#1437390908773</inState>
    <outState>Attack/TestPlans/DribbleToPointPlan.pml#1436960962060</outState>
  </transitions>
  <transitions id="1437390979982" name="MISSING_NAME" comment="anyChildSucces &amp;&amp; situation opp penalty" msg="">
    <preCondition id="1437390981105" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>Attack/TestPlans/DribbleToPointPlan.pml#1436960962060</inState>
    <outState>Attack/TestPlans/DribbleToPointPlan.pml#1437390908773</outState>
  </transitions>
  <transitions id="1437483382709" name="MISSING_NAME" comment="haveBall" msg="">
    <preCondition id="1437483384075" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>Attack/TestPlans/DribbleToPointPlan.pml#1437483357988</inState>
    <outState>Attack/TestPlans/DribbleToPointPlan.pml#1437390908773</outState>
  </transitions>
  <transitions id="1437483384370" name="MISSING_NAME" comment="lostBall" msg="">
    <preCondition id="1437483384840" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>Attack/TestPlans/DribbleToPointPlan.pml#1437390908773</inState>
    <outState>Attack/TestPlans/DribbleToPointPlan.pml#1437483357988</outState>
  </transitions>
  <transitions id="1437483408716" name="MISSING_NAME" comment="haveBall" msg="">
    <preCondition id="1437483410098" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>Attack/TestPlans/DribbleToPointPlan.pml#1437483395818</inState>
    <outState>Attack/TestPlans/DribbleToPointPlan.pml#1436960962060</outState>
  </transitions>
  <transitions id="1437483410283" name="MISSING_NAME" comment="lostBall" msg="">
    <preCondition id="1437483411068" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>Attack/TestPlans/DribbleToPointPlan.pml#1436960962060</inState>
    <outState>Attack/TestPlans/DribbleToPointPlan.pml#1437483395818</outState>
  </transitions>
  <entryPoints id="1436960854733" name="MISSING_NAME" comment="" successRequired="false" minCardinality="0" maxCardinality="2147483647">
    <task>taskrepository.tsk#1225112227903</task>
    <state>Attack/TestPlans/DribbleToPointPlan.pml#1437390908773</state>
  </entryPoints>
</alica:Plan>
