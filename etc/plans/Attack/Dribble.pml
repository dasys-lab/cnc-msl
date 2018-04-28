<?xml version="1.0" encoding="ASCII"?>
<alica:Plan xmi:version="2.0" xmlns:xmi="http://www.omg.org/XMI" xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance" xmlns:alica="http:///de.uni_kassel.vs.cn" id="1434049476066" name="Dribble" comment="" masterPlan="false" utilityFunction="" utilityThreshold="0.1" destinationPath="Plans/Attack" priority="0.0" minCardinality="0" maxCardinality="2147483647">
  <conditions xsi:type="alica:RuntimeCondition" id="1434116267322" name="NewRuntimeCondition" comment="haveBall" conditionString="" pluginName="DefaultPlugin"/>
  <states id="1434049476067" name="Dribble" comment="" entryPoint="1434049476068">
    <plans xsi:type="alica:BehaviourConfiguration">DribbleEmergencyKick.beh#1457706826895</plans>
    <plans xsi:type="alica:BehaviourConfiguration">../TestPlans/DribbleControlTest/DribbleControlMOS.beh#1479905216821</plans>
    <plans xsi:type="alica:BehaviourConfiguration">DribbleAttackConservative.beh#1457967385543</plans>
    <inTransitions>#1434050639119</inTransitions>
    <inTransitions>#1434050649347</inTransitions>
    <outTransitions>#1434050619363</outTransitions>
    <outTransitions>#1434050647042</outTransitions>
    <outTransitions>#1434050656332</outTransitions>
    <outTransitions>#1434050674494</outTransitions>
  </states>
  <states id="1434050474119" name="AlignToGoal" comment="">
    <plans xsi:type="alica:BehaviourConfiguration">../Behaviours/AlignToGoal.beh#1415205285582</plans>
    <plans xsi:type="alica:BehaviourConfiguration">../GenericBehaviours/CheckGoalKick.beh#1449076029919</plans>
    <plans xsi:type="alica:BehaviourConfiguration">../TestPlans/DribbleControlTest/DribbleControlMOS.beh#1479905216821</plans>
    <inTransitions>#1434050619363</inTransitions>
    <outTransitions>#1434050630827</outTransitions>
  </states>
  <states id="1434050502701" name="AttackAgain" comment="">
    <plans xsi:type="alica:BehaviourConfiguration">DribbleEmergencyKick.beh#1457706826895</plans>
    <plans xsi:type="alica:BehaviourConfiguration">DribbleToAttackPointConservative.beh#1458132905432</plans>
    <plans xsi:type="alica:BehaviourConfiguration">../TestPlans/DribbleControlTest/DribbleControlMOS.beh#1479905216821</plans>
    <inTransitions>#1434050630827</inTransitions>
    <inTransitions>#1434050650481</inTransitions>
    <inTransitions>#1434050656332</inTransitions>
    <inTransitions>#1434050681521</inTransitions>
    <outTransitions>#1434050639119</outTransitions>
    <outTransitions>#1434050655141</outTransitions>
    <outTransitions>#1434050686620</outTransitions>
  </states>
  <states id="1434050522682" name="ProtectBall" comment="">
    <plans xsi:type="alica:BehaviourConfiguration">ProtectBall.beh#1457706612268</plans>
    <plans xsi:type="alica:BehaviourConfiguration">DribbleEmergencyKick.beh#1457706895442</plans>
    <plans xsi:type="alica:BehaviourConfiguration">../TestPlans/DribbleControlTest/DribbleControlMOS.beh#1479905216821</plans>
    <inTransitions>#1434050674494</inTransitions>
    <inTransitions>#1434050686620</inTransitions>
    <outTransitions>#1434050681521</outTransitions>
  </states>
  <states id="1434050541225" name="TurnOneEighty" comment="">
    <plans xsi:type="alica:BehaviourConfiguration">OneEighty.beh#1434650910857</plans>
    <plans xsi:type="alica:BehaviourConfiguration">../TestPlans/DribbleControlTest/DribbleControlMOS.beh#1479905216821</plans>
    <inTransitions>#1434050647042</inTransitions>
    <inTransitions>#1434050655141</inTransitions>
    <outTransitions>#1434050649347</outTransitions>
    <outTransitions>#1434050650481</outTransitions>
  </states>
  <transitions id="1434050619363" name="MISSING_NAME" comment="Comment&#x9;may score &amp;&amp; Goal closer than PreciseShot.MaxDistance &amp;&amp; freeGoalVector exists &amp;&amp; noobstacle in turn radius &amp;&amp; Distance to goal larger than PrecideShot.MinDistance &amp;&amp; AngleToGoal larger than 10degrees" msg="">
    <preCondition id="1434050620829" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>#1434049476067</inState>
    <outState>#1434050474119</outState>
  </transitions>
  <transitions id="1434050630827" name="MISSING_NAME" comment="Fail || couldn't kick" msg="">
    <preCondition id="1434050638814" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>#1434050474119</inState>
    <outState>#1434050502701</outState>
  </transitions>
  <transitions id="1434050639119" name="MISSING_NAME" comment="success" msg="">
    <preCondition id="1434050643664" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>#1434050502701</inState>
    <outState>#1434049476067</outState>
  </transitions>
  <transitions id="1434050647042" name="MISSING_NAME" comment="Fail" msg="">
    <preCondition id="1434050649090" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>#1434049476067</inState>
    <outState>#1434050541225</outState>
  </transitions>
  <transitions id="1434050649347" name="MISSING_NAME" comment="Success &amp;&amp; Angle to goal &lt; 90 degrees" msg="">
    <preCondition id="1434050650300" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>#1434050541225</inState>
    <outState>#1434049476067</outState>
  </transitions>
  <transitions id="1434050650481" name="MISSING_NAME" comment="(Success &amp;&amp; angle to goal > 90 degrees) or Fail" msg="">
    <preCondition id="1434050655008" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>#1434050541225</inState>
    <outState>#1434050502701</outState>
  </transitions>
  <transitions id="1434050655141" name="MISSING_NAME" comment="Fail" msg="">
    <preCondition id="1434050656151" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>#1434050502701</inState>
    <outState>#1434050541225</outState>
  </transitions>
  <transitions id="1434050656332" name="MISSING_NAME" comment="ownPos.X > fieldLength/2 - goalAreaX - 1300 &amp;&amp; Abs(ownPos.Y) &lt; penaltyArea.Y" msg="">
    <preCondition id="1434050674307" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>#1434049476067</inState>
    <outState>#1434050502701</outState>
  </transitions>
  <transitions id="1434050674494" name="MISSING_NAME" comment="ProtectBall opponent is in front of us || not mayscore" msg="">
    <preCondition id="1434050677358" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>#1434049476067</inState>
    <outState>#1434050522682</outState>
  </transitions>
  <transitions id="1434050681521" name="MISSING_NAME" comment="any child success" msg="">
    <preCondition id="1434050685640" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>#1434050522682</inState>
    <outState>#1434050502701</outState>
  </transitions>
  <transitions id="1434050686620" name="MISSING_NAME" comment="ProtectBall opponent is in front of us" msg="">
    <preCondition id="1434050690800" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>#1434050502701</inState>
    <outState>#1434050522682</outState>
  </transitions>
  <entryPoints id="1434049476068" name="Attack" comment="" successRequired="false" minCardinality="0" maxCardinality="2147483647">
    <task>../../Misc/taskrepository.tsk#1222613952469</task>
    <state>#1434049476067</state>
  </entryPoints>
</alica:Plan>
