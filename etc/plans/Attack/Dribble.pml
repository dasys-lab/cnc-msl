<?xml version="1.0" encoding="ASCII"?>
<alica:Plan xmi:version="2.0" xmlns:xmi="http://www.omg.org/XMI" xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance" xmlns:alica="http:///de.uni_kassel.vs.cn" id="1434049476066" name="Dribble" comment="" masterPlan="false" utilityFunction="" utilityThreshold="0.1" destinationPath="Plans/Attack" priority="0.0" minCardinality="0" maxCardinality="2147483647">
  <conditions xsi:type="alica:RuntimeCondition" id="1434116267322" name="NewRuntimeCondition" comment="haveBall" conditionString="" pluginName="DefaultPlugin"/>
  <states id="1434049476067" name="Dribble" comment="" entryPoint="1434049476068">
    <plans xsi:type="alica:BehaviourConfiguration">DribbleToAttackPoint.beh#1436855860607</plans>
    <inTransitions>#1434050622698</inTransitions>
    <inTransitions>#1434050639119</inTransitions>
    <inTransitions>#1434050649347</inTransitions>
    <inTransitions>#1434050679613</inTransitions>
    <outTransitions>#1434050619363</outTransitions>
    <outTransitions>#1434050647042</outTransitions>
    <outTransitions>#1434050656332</outTransitions>
    <outTransitions>#1434050674494</outTransitions>
    <outTransitions>#1434050677561</outTransitions>
  </states>
  <states id="1434050474119" name="AlignToGoal" comment="">
    <inTransitions>#1434050619363</inTransitions>
    <outTransitions>#1434050621017</outTransitions>
    <outTransitions>#1434050630827</outTransitions>
  </states>
  <states id="1434050491040" name="Kick" comment="">
    <inTransitions>#1434050621017</inTransitions>
    <outTransitions>#1434050622698</outTransitions>
    <outTransitions>#1434050628706</outTransitions>
  </states>
  <states id="1434050502701" name="AttackAgain" comment="">
    <plans xsi:type="alica:BehaviourConfiguration">../Behaviours/GetBall.beh#1414840399972</plans>
    <inTransitions>#1434050628706</inTransitions>
    <inTransitions>#1434050630827</inTransitions>
    <inTransitions>#1434050650481</inTransitions>
    <inTransitions>#1434050656332</inTransitions>
    <inTransitions>#1434050681521</inTransitions>
    <outTransitions>#1434050639119</outTransitions>
    <outTransitions>#1434050655141</outTransitions>
    <outTransitions>#1434050686620</outTransitions>
  </states>
  <states id="1434050522682" name="ProtectBall" comment="">
    <inTransitions>#1434050674494</inTransitions>
    <inTransitions>#1434050686620</inTransitions>
    <outTransitions>#1434050681521</outTransitions>
  </states>
  <states id="1434050541225" name="TurnOneEighty" comment="">
    <plans xsi:type="alica:BehaviourConfiguration">OneEighty.beh#1434650910857</plans>
    <inTransitions>#1434050647042</inTransitions>
    <inTransitions>#1434050655141</inTransitions>
    <outTransitions>#1434050649347</outTransitions>
    <outTransitions>#1434050650481</outTransitions>
  </states>
  <states id="1434050576120" name="DriveBackwards" comment="">
    <inTransitions>#1434050677561</inTransitions>
    <outTransitions>#1434050679613</outTransitions>
  </states>
  <transitions id="1434050619363" name="MISSING_NAME" comment="" msg="">
    <preCondition id="1434050620829" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>#1434049476067</inState>
    <outState>#1434050474119</outState>
  </transitions>
  <transitions id="1434050621017" name="MISSING_NAME" comment="" msg="">
    <preCondition id="1434050622353" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>#1434050474119</inState>
    <outState>#1434050491040</outState>
  </transitions>
  <transitions id="1434050622698" name="MISSING_NAME" comment="" msg="">
    <preCondition id="1434050626634" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>#1434050491040</inState>
    <outState>#1434049476067</outState>
  </transitions>
  <transitions id="1434050628706" name="MISSING_NAME" comment="" msg="">
    <preCondition id="1434050630537" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>#1434050491040</inState>
    <outState>#1434050502701</outState>
  </transitions>
  <transitions id="1434050630827" name="MISSING_NAME" comment="" msg="">
    <preCondition id="1434050638814" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>#1434050474119</inState>
    <outState>#1434050502701</outState>
  </transitions>
  <transitions id="1434050639119" name="MISSING_NAME" comment="" msg="">
    <preCondition id="1434050643664" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>#1434050502701</inState>
    <outState>#1434049476067</outState>
  </transitions>
  <transitions id="1434050647042" name="MISSING_NAME" comment="" msg="">
    <preCondition id="1434050649090" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>#1434049476067</inState>
    <outState>#1434050541225</outState>
  </transitions>
  <transitions id="1434050649347" name="MISSING_NAME" comment="" msg="">
    <preCondition id="1434050650300" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>#1434050541225</inState>
    <outState>#1434049476067</outState>
  </transitions>
  <transitions id="1434050650481" name="MISSING_NAME" comment="" msg="">
    <preCondition id="1434050655008" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>#1434050541225</inState>
    <outState>#1434050502701</outState>
  </transitions>
  <transitions id="1434050655141" name="MISSING_NAME" comment="" msg="">
    <preCondition id="1434050656151" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>#1434050502701</inState>
    <outState>#1434050541225</outState>
  </transitions>
  <transitions id="1434050656332" name="MISSING_NAME" comment="" msg="">
    <preCondition id="1434050674307" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>#1434049476067</inState>
    <outState>#1434050502701</outState>
  </transitions>
  <transitions id="1434050674494" name="MISSING_NAME" comment="" msg="">
    <preCondition id="1434050677358" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>#1434049476067</inState>
    <outState>#1434050522682</outState>
  </transitions>
  <transitions id="1434050677561" name="MISSING_NAME" comment="" msg="">
    <preCondition id="1434050679354" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>#1434049476067</inState>
    <outState>#1434050576120</outState>
  </transitions>
  <transitions id="1434050679613" name="MISSING_NAME" comment="" msg="">
    <preCondition id="1434050681289" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>#1434050576120</inState>
    <outState>#1434049476067</outState>
  </transitions>
  <transitions id="1434050681521" name="MISSING_NAME" comment="" msg="">
    <preCondition id="1434050685640" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>#1434050522682</inState>
    <outState>#1434050502701</outState>
  </transitions>
  <transitions id="1434050686620" name="MISSING_NAME" comment="" msg="">
    <preCondition id="1434050690800" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>#1434050502701</inState>
    <outState>#1434050522682</outState>
  </transitions>
  <entryPoints id="1434049476068" name="Attack" comment="" successRequired="false" minCardinality="0" maxCardinality="2147483647">
    <task>../../Misc/taskrepository.tsk#1222613952469</task>
    <state>#1434049476067</state>
  </entryPoints>
</alica:Plan>
