<?xml version="1.0" encoding="ASCII"?>
<alica:Plan xmi:version="2.0" xmlns:xmi="http://www.omg.org/XMI" xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance" xmlns:alica="http:///de.uni_kassel.vs.cn" id="1518622787399" name="TestDribbleMOS" comment="" masterPlan="true" utilityFunction="" utilityThreshold="0.1" destinationPath="" priority="0.0" minCardinality="0" maxCardinality="2147483647">
  <states id="1518622787400" name="NewState" comment="" entryPoint="1518622787401">
    <inTransitions>#1518622829104</inTransitions>
    <inTransitions>#1518622830792</inTransitions>
    <inTransitions>#1518622836117</inTransitions>
    <inTransitions>#1518622839547</inTransitions>
    <inTransitions>#1518622846236</inTransitions>
    <inTransitions>#1518623968955</inTransitions>
    <outTransitions>#1518622827729</outTransitions>
    <outTransitions>#1518622834600</outTransitions>
    <outTransitions>#1518622838067</outTransitions>
    <outTransitions>#1518622842678</outTransitions>
    <outTransitions>#1518622844428</outTransitions>
    <outTransitions>#1518623967163</outTransitions>
  </states>
  <states id="1518622802933" name="DriveZigZagViewZigZag" comment="">
    <plans xsi:type="alica:Plan">DriveZigZagViewZigZag.pml#1518623186077</plans>
    <inTransitions>#1518622827729</inTransitions>
    <outTransitions>#1518622829104</outTransitions>
  </states>
  <states id="1518622804332" name="DriveZigZagViewStraight" comment="">
    <plans xsi:type="alica:Plan">DriveZigZagViewStraight.pml#1518623233217</plans>
    <inTransitions>#1518622834600</inTransitions>
    <outTransitions>#1518622830792</outTransitions>
  </states>
  <states id="1518622806277" name="CircleCenter" comment="">
    <inTransitions>#1518622838067</inTransitions>
    <outTransitions>#1518622836117</outTransitions>
  </states>
  <states id="1518622811110" name="CircleLine" comment="">
    <inTransitions>#1518622842678</inTransitions>
    <outTransitions>#1518622839547</outTransitions>
  </states>
  <states id="1518622824671" name="Square" comment="">
    <inTransitions>#1518622844428</inTransitions>
    <outTransitions>#1518622846236</outTransitions>
  </states>
  <states id="1518623882447" name="DriveStraightViewZigZag" comment="">
    <plans xsi:type="alica:Plan">DriveStraightViewZigZag.pml#1518628181064</plans>
    <inTransitions>#1518623967163</inTransitions>
    <outTransitions>#1518623968955</outTransitions>
  </states>
  <transitions id="1518622827729" name="MISSING_NAME" comment="" msg="">
    <preCondition id="1518622828944" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>#1518622787400</inState>
    <outState>#1518622802933</outState>
  </transitions>
  <transitions id="1518622829104" name="MISSING_NAME" comment="" msg="">
    <preCondition id="1518622830640" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>#1518622802933</inState>
    <outState>#1518622787400</outState>
  </transitions>
  <transitions id="1518622830792" name="MISSING_NAME" comment="" msg="">
    <preCondition id="1518622834399" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>#1518622804332</inState>
    <outState>#1518622787400</outState>
  </transitions>
  <transitions id="1518622834600" name="MISSING_NAME" comment="" msg="">
    <preCondition id="1518622835957" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>#1518622787400</inState>
    <outState>#1518622804332</outState>
  </transitions>
  <transitions id="1518622836117" name="MISSING_NAME" comment="" msg="">
    <preCondition id="1518622837786" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>#1518622806277</inState>
    <outState>#1518622787400</outState>
  </transitions>
  <transitions id="1518622838067" name="MISSING_NAME" comment="" msg="">
    <preCondition id="1518622839323" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>#1518622787400</inState>
    <outState>#1518622806277</outState>
  </transitions>
  <transitions id="1518622839547" name="MISSING_NAME" comment="" msg="">
    <preCondition id="1518622842509" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>#1518622811110</inState>
    <outState>#1518622787400</outState>
  </transitions>
  <transitions id="1518622842678" name="MISSING_NAME" comment="" msg="">
    <preCondition id="1518622844227" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>#1518622787400</inState>
    <outState>#1518622811110</outState>
  </transitions>
  <transitions id="1518622844428" name="MISSING_NAME" comment="" msg="">
    <preCondition id="1518622846035" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>#1518622787400</inState>
    <outState>#1518622824671</outState>
  </transitions>
  <transitions id="1518622846236" name="MISSING_NAME" comment="" msg="">
    <preCondition id="1518622847338" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>#1518622824671</inState>
    <outState>#1518622787400</outState>
  </transitions>
  <transitions id="1518623967163" name="MISSING_NAME" comment="" msg="">
    <preCondition id="1518623968811" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>#1518622787400</inState>
    <outState>#1518623882447</outState>
  </transitions>
  <transitions id="1518623968955" name="MISSING_NAME" comment="" msg="">
    <preCondition id="1518623970678" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>#1518623882447</inState>
    <outState>#1518622787400</outState>
  </transitions>
  <entryPoints id="1518622787401" name="MISSING_NAME" comment="" successRequired="false" minCardinality="0" maxCardinality="2147483647">
    <task>../../../Misc/taskrepository.tsk#1225112227903</task>
    <state>#1518622787400</state>
  </entryPoints>
</alica:Plan>
