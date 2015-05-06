<?xml version="1.0" encoding="ASCII"?>
<alica:Plan xmi:version="2.0" xmlns:xmi="http://www.omg.org/XMI" xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance" xmlns:alica="http:///de.uni_kassel.vs.cn" id="1430924951132" name="GenericOwnStandards" comment="" masterPlan="false" utilityFunction="" utilityThreshold="0.1" destinationPath="Plans/GenericStandards" priority="0.0" minCardinality="0" maxCardinality="2147483647">
  <states id="1430924951133" name="Dummy" comment="" entryPoint="1430925626532">
    <outTransitions>#1430925917001</outTransitions>
    <outTransitions>#1430925918585</outTransitions>
    <outTransitions>#1430925919866</outTransitions>
    <outTransitions>#1430925921331</outTransitions>
    <outTransitions>#1430925922843</outTransitions>
    <outTransitions>#1430925924151</outTransitions>
  </states>
  <states id="1430925718512" name="OwnThrowIn" comment="">
    <inTransitions>#1430925917001</inTransitions>
    <outTransitions>#1430925960854</outTransitions>
  </states>
  <states id="1430925736746" name="OwnKickOff" comment="">
    <inTransitions>#1430925918585</inTransitions>
    <outTransitions>#1430925962882</outTransitions>
  </states>
  <states id="1430925743739" name="OwnGoalKick" comment="">
    <inTransitions>#1430925919866</inTransitions>
    <outTransitions>#1430925965662</outTransitions>
  </states>
  <states id="1430925751875" name="OwnFreeKick" comment="">
    <inTransitions>#1430925922843</inTransitions>
    <outTransitions>#1430925971330</outTransitions>
  </states>
  <states id="1430925759928" name="OwnCornerKick" comment="">
    <inTransitions>#1430925924151</inTransitions>
    <outTransitions>#1430925973111</outTransitions>
  </states>
  <states id="1430925774870" name="OwnPenaltyKick" comment="">
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
  <transitions id="1430925960854" name="MISSING_NAME" comment="any child success" msg="">
    <preCondition id="1430925962659" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>#1430925718512</inState>
    <outState>#1430925945981</outState>
  </transitions>
  <transitions id="1430925962882" name="MISSING_NAME" comment="any child success" msg="">
    <preCondition id="1430925965501" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>#1430925736746</inState>
    <outState>#1430925945981</outState>
  </transitions>
  <transitions id="1430925965662" name="MISSING_NAME" comment="any child success" msg="">
    <preCondition id="1430925967520" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>#1430925743739</inState>
    <outState>#1430925945981</outState>
  </transitions>
  <transitions id="1430925967608" name="MISSING_NAME" comment="any child success" msg="">
    <preCondition id="1430925970967" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>#1430925774870</inState>
    <outState>#1430925945981</outState>
  </transitions>
  <transitions id="1430925971330" name="MISSING_NAME" comment="any child success" msg="">
    <preCondition id="1430925972838" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>#1430925751875</inState>
    <outState>#1430925945981</outState>
  </transitions>
  <transitions id="1430925973111" name="MISSING_NAME" comment="any child success" msg="">
    <preCondition id="1430925975558" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>#1430925759928</inState>
    <outState>#1430925945981</outState>
  </transitions>
  <entryPoints id="1430925626532" name="NewEntryPoint" comment="" successRequired="false" minCardinality="0" maxCardinality="2147483647">
    <task>../../Misc/taskrepository.tsk#1225112227903</task>
    <state>#1430924951133</state>
  </entryPoints>
</alica:Plan>
