<?xml version="1.0" encoding="ASCII"?>
<alica:Plan xmi:version="2.0" xmlns:xmi="http://www.omg.org/XMI" xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance" xmlns:alica="http:///de.uni_kassel.vs.cn" id="1518624635783" name="CircleCenter" comment="" masterPlan="false" utilityFunction="" utilityThreshold="0.1" destinationPath="" priority="0.0" minCardinality="0" maxCardinality="2147483647">
  <states id="1518624635784" name="N" comment="" entryPoint="1518624635785">
    <plans xsi:type="alica:BehaviourConfiguration">../../GenericBehaviours/DriveAndAlignToPoint.beh#1518625061828</plans>
    <outTransitions>#1518624920724</outTransitions>
  </states>
  <states id="1518624878623" name="NO" comment="">
    <plans xsi:type="alica:BehaviourConfiguration">../../GenericBehaviours/DriveAndAlignToPoint.beh#1518625117909</plans>
    <inTransitions>#1518624920724</inTransitions>
    <outTransitions>#1518624922558</outTransitions>
  </states>
  <states id="1518624881576" name="NOO" comment="">
    <plans xsi:type="alica:BehaviourConfiguration">../../GenericBehaviours/DriveAndAlignToPoint.beh#1518625653420</plans>
    <inTransitions>#1518624922558</inTransitions>
    <outTransitions>#1518624923729</outTransitions>
  </states>
  <states id="1518624884409" name="O" comment="">
    <plans xsi:type="alica:BehaviourConfiguration">../../GenericBehaviours/DriveAndAlignToPoint.beh#1518625824531</plans>
    <inTransitions>#1518624923729</inTransitions>
    <outTransitions>#1518624924935</outTransitions>
  </states>
  <states id="1518624886995" name="SOO" comment="">
    <plans xsi:type="alica:BehaviourConfiguration">../../GenericBehaviours/DriveAndAlignToPoint.beh#1518625669215</plans>
    <inTransitions>#1518624924935</inTransitions>
    <outTransitions>#1518624926045</outTransitions>
  </states>
  <states id="1518624889603" name="SO" comment="">
    <plans xsi:type="alica:BehaviourConfiguration">../../GenericBehaviours/DriveAndAlignToPoint.beh#1518625156903</plans>
    <inTransitions>#1518624926045</inTransitions>
    <outTransitions>#1518624927256</outTransitions>
  </states>
  <states id="1518624892228" name="S" comment="">
    <plans xsi:type="alica:BehaviourConfiguration">../../GenericBehaviours/DriveAndAlignToPoint.beh#1518625802968</plans>
    <inTransitions>#1518624927256</inTransitions>
    <outTransitions>#1518624928667</outTransitions>
  </states>
  <states id="1518624896101" name="SW" comment="">
    <plans xsi:type="alica:BehaviourConfiguration">../../GenericBehaviours/DriveAndAlignToPoint.beh#1518625265141</plans>
    <inTransitions>#1518624928667</inTransitions>
    <outTransitions>#1518624930014</outTransitions>
  </states>
  <states id="1518624898694" name="SWW" comment="">
    <plans xsi:type="alica:BehaviourConfiguration">../../GenericBehaviours/DriveAndAlignToPoint.beh#1518625684021</plans>
    <inTransitions>#1518624930014</inTransitions>
    <outTransitions>#1518624931187</outTransitions>
  </states>
  <states id="1518624900831" name="W" comment="">
    <plans xsi:type="alica:BehaviourConfiguration">../../GenericBehaviours/DriveAndAlignToPoint.beh#1518625721179</plans>
    <inTransitions>#1518624931187</inTransitions>
    <outTransitions>#1518624932479</outTransitions>
  </states>
  <states id="1518624902640" name="NWW" comment="">
    <plans xsi:type="alica:BehaviourConfiguration">../../GenericBehaviours/DriveAndAlignToPoint.beh#1518625698001</plans>
    <inTransitions>#1518624932479</inTransitions>
    <outTransitions>#1518624933459</outTransitions>
  </states>
  <states id="1518624906169" name="NW" comment="">
    <plans xsi:type="alica:BehaviourConfiguration">../../GenericBehaviours/DriveAndAlignToPoint.beh#1518625189001</plans>
    <inTransitions>#1518624933459</inTransitions>
    <outTransitions>#1518624934408</outTransitions>
  </states>
  <states id="1518624910218" name="N2" comment="">
    <plans xsi:type="alica:BehaviourConfiguration">../../GenericBehaviours/DriveAndAlignToPoint.beh#1518625061828</plans>
    <inTransitions>#1518624934408</inTransitions>
    <outTransitions>#1518624968273</outTransitions>
  </states>
  <states xsi:type="alica:SuccessState" id="1518624915083" name="NewSuccessState" comment="">
    <inTransitions>#1518624968273</inTransitions>
  </states>
  <transitions id="1518624920724" name="MISSING_NAME" comment="" msg="">
    <preCondition id="1518624922421" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>#1518624635784</inState>
    <outState>#1518624878623</outState>
  </transitions>
  <transitions id="1518624922558" name="MISSING_NAME" comment="" msg="">
    <preCondition id="1518624923528" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>#1518624878623</inState>
    <outState>#1518624881576</outState>
  </transitions>
  <transitions id="1518624923729" name="MISSING_NAME" comment="" msg="">
    <preCondition id="1518624924742" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>#1518624881576</inState>
    <outState>#1518624884409</outState>
  </transitions>
  <transitions id="1518624924935" name="MISSING_NAME" comment="" msg="">
    <preCondition id="1518624925908" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>#1518624884409</inState>
    <outState>#1518624886995</outState>
  </transitions>
  <transitions id="1518624926045" name="MISSING_NAME" comment="" msg="">
    <preCondition id="1518624927095" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>#1518624886995</inState>
    <outState>#1518624889603</outState>
  </transitions>
  <transitions id="1518624927256" name="MISSING_NAME" comment="" msg="">
    <preCondition id="1518624928426" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>#1518624889603</inState>
    <outState>#1518624892228</outState>
  </transitions>
  <transitions id="1518624928667" name="MISSING_NAME" comment="" msg="">
    <preCondition id="1518624929829" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>#1518624892228</inState>
    <outState>#1518624896101</outState>
  </transitions>
  <transitions id="1518624930014" name="MISSING_NAME" comment="" msg="">
    <preCondition id="1518624930978" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>#1518624896101</inState>
    <outState>#1518624898694</outState>
  </transitions>
  <transitions id="1518624931187" name="MISSING_NAME" comment="" msg="">
    <preCondition id="1518624932238" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>#1518624898694</inState>
    <outState>#1518624900831</outState>
  </transitions>
  <transitions id="1518624932479" name="MISSING_NAME" comment="" msg="">
    <preCondition id="1518624933146" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>#1518624900831</inState>
    <outState>#1518624902640</outState>
  </transitions>
  <transitions id="1518624933459" name="MISSING_NAME" comment="" msg="">
    <preCondition id="1518624934271" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>#1518624902640</inState>
    <outState>#1518624906169</outState>
  </transitions>
  <transitions id="1518624934408" name="MISSING_NAME" comment="" msg="">
    <preCondition id="1518624935190" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>#1518624906169</inState>
    <outState>#1518624910218</outState>
  </transitions>
  <transitions id="1518624968273" name="MISSING_NAME" comment="" msg="">
    <preCondition id="1518624970684" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>#1518624910218</inState>
    <outState>#1518624915083</outState>
  </transitions>
  <entryPoints id="1518624635785" name="MISSING_NAME" comment="" successRequired="false" minCardinality="0" maxCardinality="2147483647">
    <task>../../../Misc/taskrepository.tsk#1225112227903</task>
    <state>#1518624635784</state>
  </entryPoints>
</alica:Plan>
