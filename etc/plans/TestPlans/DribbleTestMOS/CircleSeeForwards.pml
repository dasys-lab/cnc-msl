<?xml version="1.0" encoding="ASCII"?>
<alica:Plan xmi:version="2.0" xmlns:xmi="http://www.omg.org/XMI" xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance" xmlns:alica="http:///de.uni_kassel.vs.cn" id="1520679916966" name="CircleSeeForwards" comment="" masterPlan="false" utilityFunction="" utilityThreshold="0.1" destinationPath="Plans/TestPlans/DribbleTestMOS" priority="0.0" minCardinality="0" maxCardinality="2147483647">
  <states id="1520679916967" name="N" comment="" entryPoint="1520679916968">
    <plans xsi:type="alica:BehaviourConfiguration">../../GenericBehaviours/DriveAndAlignToPoint.beh#1517050799588</plans>
    <outTransitions>#1520682657286</outTransitions>
  </states>
  <states id="1520681093152" name="NO" comment="">
    <plans xsi:type="alica:BehaviourConfiguration">../../GenericBehaviours/DriveAndAlignToPoint.beh#1520682100349</plans>
    <inTransitions>#1520682657286</inTransitions>
    <outTransitions>#1520682661860</outTransitions>
  </states>
  <states id="1520681096472" name="NOO" comment="">
    <plans xsi:type="alica:BehaviourConfiguration">../../GenericBehaviours/DriveAndAlignToPoint.beh#1520682426424</plans>
    <inTransitions>#1520682661860</inTransitions>
    <outTransitions>#1520682664654</outTransitions>
  </states>
  <states id="1520681099138" name="O" comment="">
    <plans xsi:type="alica:BehaviourConfiguration">../../GenericBehaviours/DriveAndAlignToPoint.beh#1520682353487</plans>
    <inTransitions>#1520682664654</inTransitions>
    <outTransitions>#1520682666508</outTransitions>
  </states>
  <states id="1520681101802" name="SOO" comment="">
    <plans xsi:type="alica:BehaviourConfiguration">../../GenericBehaviours/DriveAndAlignToPoint.beh#1520682461896</plans>
    <inTransitions>#1520682666508</inTransitions>
    <outTransitions>#1520682668045</outTransitions>
  </states>
  <states id="1520681103972" name="SO" comment="">
    <plans xsi:type="alica:BehaviourConfiguration">../../GenericBehaviours/DriveAndAlignToPoint.beh#1520682164956</plans>
    <inTransitions>#1520682668045</inTransitions>
    <outTransitions>#1520682669912</outTransitions>
  </states>
  <states id="1520681107564" name="S" comment="">
    <plans xsi:type="alica:BehaviourConfiguration">../../GenericBehaviours/DriveAndAlignToPoint.beh#1517054288108</plans>
    <inTransitions>#1520682669912</inTransitions>
    <outTransitions>#1520682671361</outTransitions>
  </states>
  <states id="1520681111037" name="SW" comment="">
    <plans xsi:type="alica:BehaviourConfiguration">../../GenericBehaviours/DriveAndAlignToPoint.beh#1520682200024</plans>
    <inTransitions>#1520682671361</inTransitions>
    <outTransitions>#1520682672926</outTransitions>
  </states>
  <states id="1520681114046" name="SWW" comment="">
    <plans xsi:type="alica:BehaviourConfiguration">../../GenericBehaviours/DriveAndAlignToPoint.beh#1520682511169</plans>
    <inTransitions>#1520682672926</inTransitions>
    <outTransitions>#1520682674252</outTransitions>
  </states>
  <states id="1520681116863" name="W" comment="">
    <plans xsi:type="alica:BehaviourConfiguration">../../GenericBehaviours/DriveAndAlignToPoint.beh#1520682373772</plans>
    <inTransitions>#1520682674252</inTransitions>
    <outTransitions>#1520682676410</outTransitions>
  </states>
  <states id="1520681120032" name="NWW" comment="">
    <plans xsi:type="alica:BehaviourConfiguration">../../GenericBehaviours/DriveAndAlignToPoint.beh#1520682534452</plans>
    <inTransitions>#1520682676410</inTransitions>
    <outTransitions>#1520682677552</outTransitions>
  </states>
  <states id="1520681122553" name="NW" comment="">
    <plans xsi:type="alica:BehaviourConfiguration">../../GenericBehaviours/DriveAndAlignToPoint.beh#1520682118655</plans>
    <inTransitions>#1520682677552</inTransitions>
    <outTransitions>#1520682678937</outTransitions>
  </states>
  <states id="1520681125362" name="N" comment="">
    <plans xsi:type="alica:BehaviourConfiguration">../../GenericBehaviours/DriveAndAlignToPoint.beh#1517050799588</plans>
    <inTransitions>#1520682678937</inTransitions>
    <outTransitions>#1520682680808</outTransitions>
  </states>
  <states xsi:type="alica:SuccessState" id="1520681525669" name="NewSuccessState" comment="">
    <inTransitions>#1520682680808</inTransitions>
  </states>
  <transitions id="1520682657286" name="MISSING_NAME" comment="" msg="">
    <preCondition id="1520682661242" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>#1520679916967</inState>
    <outState>#1520681093152</outState>
  </transitions>
  <transitions id="1520682661860" name="MISSING_NAME" comment="" msg="">
    <preCondition id="1520682664300" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>#1520681093152</inState>
    <outState>#1520681096472</outState>
  </transitions>
  <transitions id="1520682664654" name="MISSING_NAME" comment="" msg="">
    <preCondition id="1520682666211" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>#1520681096472</inState>
    <outState>#1520681099138</outState>
  </transitions>
  <transitions id="1520682666508" name="MISSING_NAME" comment="" msg="">
    <preCondition id="1520682667668" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>#1520681099138</inState>
    <outState>#1520681101802</outState>
  </transitions>
  <transitions id="1520682668045" name="MISSING_NAME" comment="" msg="">
    <preCondition id="1520682669615" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>#1520681101802</inState>
    <outState>#1520681103972</outState>
  </transitions>
  <transitions id="1520682669912" name="MISSING_NAME" comment="" msg="">
    <preCondition id="1520682671176" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>#1520681103972</inState>
    <outState>#1520681107564</outState>
  </transitions>
  <transitions id="1520682671361" name="MISSING_NAME" comment="" msg="">
    <preCondition id="1520682672669" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>#1520681107564</inState>
    <outState>#1520681111037</outState>
  </transitions>
  <transitions id="1520682672926" name="MISSING_NAME" comment="" msg="">
    <preCondition id="1520682674019" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>#1520681111037</inState>
    <outState>#1520681114046</outState>
  </transitions>
  <transitions id="1520682674252" name="MISSING_NAME" comment="" msg="">
    <preCondition id="1520682675912" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>#1520681114046</inState>
    <outState>#1520681116863</outState>
  </transitions>
  <transitions id="1520682676410" name="MISSING_NAME" comment="" msg="">
    <preCondition id="1520682677336" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>#1520681116863</inState>
    <outState>#1520681120032</outState>
  </transitions>
  <transitions id="1520682677552" name="MISSING_NAME" comment="" msg="">
    <preCondition id="1520682678705" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>#1520681120032</inState>
    <outState>#1520681122553</outState>
  </transitions>
  <transitions id="1520682678937" name="MISSING_NAME" comment="" msg="">
    <preCondition id="1520682680343" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>#1520681122553</inState>
    <outState>#1520681125362</outState>
  </transitions>
  <transitions id="1520682680808" name="MISSING_NAME" comment="" msg="">
    <preCondition id="1520682683152" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>#1520681125362</inState>
    <outState>#1520681525669</outState>
  </transitions>
  <entryPoints id="1520679916968" name="MISSING_NAME" comment="" successRequired="false" minCardinality="0" maxCardinality="2147483647">
    <task>../../../Misc/taskrepository.tsk#1225112227903</task>
    <state>#1520679916967</state>
  </entryPoints>
</alica:Plan>
