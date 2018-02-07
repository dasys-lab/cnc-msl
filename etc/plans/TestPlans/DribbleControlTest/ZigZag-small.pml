<?xml version="1.0" encoding="ASCII"?>
<alica:Plan xmi:version="2.0" xmlns:xmi="http://www.omg.org/XMI" xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance" xmlns:alica="http:///de.uni_kassel.vs.cn" id="1517050419596" name="ZigZag-small" comment="" masterPlan="false" utilityFunction="" utilityThreshold="0.1" destinationPath="" priority="0.0" minCardinality="0" maxCardinality="2147483647">
  <states id="1517050590881" name="Start" comment="" entryPoint="1517050590882">
    <plans xsi:type="alica:BehaviourConfiguration">../../GenericBehaviours/DriveAndAlignToPoint.beh#1517050799588</plans>
    <outTransitions>#1517413167929</outTransitions>
  </states>
  <states id="1517050976433" name="Zig1" comment="">
    <plans xsi:type="alica:BehaviourConfiguration">../../GenericBehaviours/DriveAndAlignToPoint.beh#1517052566683</plans>
    <inTransitions>#1517413167929</inTransitions>
    <outTransitions>#1517413170411</outTransitions>
  </states>
  <states id="1517052678028" name="Zag1" comment="">
    <plans xsi:type="alica:BehaviourConfiguration">../../GenericBehaviours/DriveAndAlignToPoint.beh#1517052704566</plans>
    <inTransitions>#1517413170411</inTransitions>
    <outTransitions>#1517413172014</outTransitions>
  </states>
  <states id="1517054064073" name="Zig2" comment="">
    <plans xsi:type="alica:BehaviourConfiguration">../../GenericBehaviours/DriveAndAlignToPoint.beh#1517054090420</plans>
    <inTransitions>#1517413172014</inTransitions>
    <outTransitions>#1517413173421</outTransitions>
  </states>
  <states id="1517054141087" name="Zag2" comment="">
    <plans xsi:type="alica:BehaviourConfiguration">../../GenericBehaviours/DriveAndAlignToPoint.beh#1517054165498</plans>
    <inTransitions>#1517413173421</inTransitions>
    <outTransitions>#1517413174672</outTransitions>
  </states>
  <states id="1517054206331" name="Zig3" comment="">
    <plans xsi:type="alica:BehaviourConfiguration">../../GenericBehaviours/DriveAndAlignToPoint.beh#1517054241072</plans>
    <inTransitions>#1517413174672</inTransitions>
    <outTransitions>#1517413175878</outTransitions>
  </states>
  <states id="1517054216500" name="Zag3" comment="">
    <plans xsi:type="alica:BehaviourConfiguration">../../GenericBehaviours/DriveAndAlignToPoint.beh#1517054288108</plans>
    <inTransitions>#1517413175878</inTransitions>
    <outTransitions>#1517413177300</outTransitions>
  </states>
  <states id="1517054331782" name="Zig4" comment="">
    <plans xsi:type="alica:BehaviourConfiguration">../../GenericBehaviours/DriveAndAlignToPoint.beh#1517054365259</plans>
    <inTransitions>#1517413177300</inTransitions>
    <outTransitions>#1517413179460</outTransitions>
  </states>
  <states id="1517054335727" name="Zag4" comment="">
    <plans xsi:type="alica:BehaviourConfiguration">../../GenericBehaviours/DriveAndAlignToPoint.beh#1517054380022</plans>
    <inTransitions>#1517413179460</inTransitions>
    <outTransitions>#1517413180941</outTransitions>
  </states>
  <states id="1517054512342" name="Zig5" comment="">
    <plans xsi:type="alica:BehaviourConfiguration">../../GenericBehaviours/DriveAndAlignToPoint.beh#1517054559436</plans>
    <inTransitions>#1517413180941</inTransitions>
    <outTransitions>#1517413183140</outTransitions>
  </states>
  <states id="1517054515718" name="Zag5" comment="">
    <plans xsi:type="alica:BehaviourConfiguration">../../GenericBehaviours/DriveAndAlignToPoint.beh#1517054583522</plans>
    <inTransitions>#1517413183140</inTransitions>
    <outTransitions>#1517413184359</outTransitions>
  </states>
  <states id="1517054713732" name="Zig6" comment="">
    <plans xsi:type="alica:BehaviourConfiguration">../../GenericBehaviours/DriveAndAlignToPoint.beh#1517054756760</plans>
    <inTransitions>#1517413184359</inTransitions>
    <outTransitions>#1517413187931</outTransitions>
  </states>
  <states id="1517054715562" name="Zag6" comment="">
    <plans xsi:type="alica:BehaviourConfiguration">../../GenericBehaviours/DriveAndAlignToPoint.beh#1517054780047</plans>
    <inTransitions>#1517413187931</inTransitions>
    <outTransitions>#1517413569061</outTransitions>
  </states>
  <states id="1517413211840" name="Zig7" comment="">
    <plans xsi:type="alica:BehaviourConfiguration">../../GenericBehaviours/DriveAndAlignToPoint.beh#1517413343977</plans>
    <inTransitions>#1517413569061</inTransitions>
    <outTransitions>#1517413570418</outTransitions>
  </states>
  <states id="1517413221297" name="Zag7" comment="">
    <plans xsi:type="alica:BehaviourConfiguration">../../GenericBehaviours/DriveAndAlignToPoint.beh#1517413374873</plans>
    <inTransitions>#1517413570418</inTransitions>
    <outTransitions>#1517413765855</outTransitions>
  </states>
  <states id="1517413576099" name="Zig8" comment="">
    <plans xsi:type="alica:BehaviourConfiguration">../../GenericBehaviours/DriveAndAlignToPoint.beh#1517413717087</plans>
    <inTransitions>#1517413765855</inTransitions>
    <outTransitions>#1517413767639</outTransitions>
  </states>
  <states id="1517413583020" name="Zag8" comment="">
    <plans xsi:type="alica:BehaviourConfiguration">../../GenericBehaviours/DriveAndAlignToPoint.beh#1517413743789</plans>
    <inTransitions>#1517413767639</inTransitions>
    <outTransitions>#1517413958270</outTransitions>
  </states>
  <states id="1517413828625" name="Zig9" comment="">
    <plans xsi:type="alica:BehaviourConfiguration">../../GenericBehaviours/DriveAndAlignToPoint.beh#1517413986109</plans>
    <inTransitions>#1517413958270</inTransitions>
    <outTransitions>#1517413959500</outTransitions>
  </states>
  <states id="1517413835602" name="Zag9" comment="">
    <plans xsi:type="alica:BehaviourConfiguration">../../GenericBehaviours/DriveAndAlignToPoint.beh#1517414007376</plans>
    <inTransitions>#1517413959500</inTransitions>
    <outTransitions>#1517414411587</outTransitions>
  </states>
  <states id="1517414374046" name="Zig10" comment="">
    <plans xsi:type="alica:BehaviourConfiguration">../../GenericBehaviours/DriveAndAlignToPoint.beh#1517414454147</plans>
    <inTransitions>#1517414411587</inTransitions>
    <outTransitions>#1517414413132</outTransitions>
  </states>
  <states id="1517414385931" name="Zag10" comment="">
    <plans xsi:type="alica:BehaviourConfiguration">../../GenericBehaviours/DriveAndAlignToPoint.beh#1517414498820</plans>
    <inTransitions>#1517414413132</inTransitions>
    <outTransitions>#1517415414091</outTransitions>
  </states>
  <states id="1517414888265" name="Zig11" comment="">
    <plans xsi:type="alica:BehaviourConfiguration">../../GenericBehaviours/DriveAndAlignToPoint.beh#1517415111859</plans>
    <inTransitions>#1517415414091</inTransitions>
    <outTransitions>#1517415421538</outTransitions>
  </states>
  <states id="1517414896747" name="Zag11" comment="">
    <plans xsi:type="alica:BehaviourConfiguration">../../GenericBehaviours/DriveAndAlignToPoint.beh#1517415170220</plans>
    <inTransitions>#1517415421538</inTransitions>
    <outTransitions>#1517415427753</outTransitions>
  </states>
  <states id="1517415180269" name="Zig12" comment="">
    <plans xsi:type="alica:BehaviourConfiguration">../../GenericBehaviours/DriveAndAlignToPoint.beh#1517415329054</plans>
    <inTransitions>#1517415427753</inTransitions>
    <outTransitions>#1517415435160</outTransitions>
  </states>
  <states id="1517415192926" name="Zag12" comment="">
    <plans xsi:type="alica:BehaviourConfiguration">../../GenericBehaviours/DriveAndAlignToPoint.beh#1517415399331</plans>
    <inTransitions>#1517415435160</inTransitions>
  </states>
  <transitions id="1517413167929" name="MISSING_NAME" comment="" msg="">
    <preCondition id="1517413169995" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>#1517050590881</inState>
    <outState>#1517050976433</outState>
  </transitions>
  <transitions id="1517413170411" name="MISSING_NAME" comment="" msg="">
    <preCondition id="1517413171869" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>#1517050976433</inState>
    <outState>#1517052678028</outState>
  </transitions>
  <transitions id="1517413172014" name="MISSING_NAME" comment="" msg="">
    <preCondition id="1517413173172" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>#1517052678028</inState>
    <outState>#1517054064073</outState>
  </transitions>
  <transitions id="1517413173421" name="MISSING_NAME" comment="" msg="">
    <preCondition id="1517413174479" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>#1517054064073</inState>
    <outState>#1517054141087</outState>
  </transitions>
  <transitions id="1517413174672" name="MISSING_NAME" comment="" msg="">
    <preCondition id="1517413175725" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>#1517054141087</inState>
    <outState>#1517054206331</outState>
  </transitions>
  <transitions id="1517413175878" name="MISSING_NAME" comment="" msg="">
    <preCondition id="1517413177115" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>#1517054206331</inState>
    <outState>#1517054216500</outState>
  </transitions>
  <transitions id="1517413177300" name="MISSING_NAME" comment="" msg="">
    <preCondition id="1517413179227" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>#1517054216500</inState>
    <outState>#1517054331782</outState>
  </transitions>
  <transitions id="1517413179460" name="MISSING_NAME" comment="" msg="">
    <preCondition id="1517413180772" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>#1517054331782</inState>
    <outState>#1517054335727</outState>
  </transitions>
  <transitions id="1517413180941" name="MISSING_NAME" comment="" msg="">
    <preCondition id="1517413182626" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>#1517054335727</inState>
    <outState>#1517054512342</outState>
  </transitions>
  <transitions id="1517413183140" name="MISSING_NAME" comment="" msg="">
    <preCondition id="1517413183798" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>#1517054512342</inState>
    <outState>#1517054515718</outState>
  </transitions>
  <transitions id="1517413184359" name="MISSING_NAME" comment="" msg="">
    <preCondition id="1517413187658" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>#1517054515718</inState>
    <outState>#1517054713732</outState>
  </transitions>
  <transitions id="1517413187931" name="MISSING_NAME" comment="" msg="">
    <preCondition id="1517413188974" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>#1517054713732</inState>
    <outState>#1517054715562</outState>
  </transitions>
  <transitions id="1517413569061" name="MISSING_NAME" comment="" msg="">
    <preCondition id="1517413570226" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>#1517054715562</inState>
    <outState>#1517413211840</outState>
  </transitions>
  <transitions id="1517413570418" name="MISSING_NAME" comment="" msg="">
    <preCondition id="1517413571237" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>#1517413211840</inState>
    <outState>#1517413221297</outState>
  </transitions>
  <transitions id="1517413765855" name="MISSING_NAME" comment="" msg="">
    <preCondition id="1517413767285" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>#1517413221297</inState>
    <outState>#1517413576099</outState>
  </transitions>
  <transitions id="1517413767639" name="MISSING_NAME" comment="" msg="">
    <preCondition id="1517413768289" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>#1517413576099</inState>
    <outState>#1517413583020</outState>
  </transitions>
  <transitions id="1517413958270" name="MISSING_NAME" comment="" msg="">
    <preCondition id="1517413959347" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>#1517413583020</inState>
    <outState>#1517413828625</outState>
  </transitions>
  <transitions id="1517413959500" name="MISSING_NAME" comment="" msg="">
    <preCondition id="1517413960211" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>#1517413828625</inState>
    <outState>#1517413835602</outState>
  </transitions>
  <transitions id="1517414411587" name="MISSING_NAME" comment="" msg="">
    <preCondition id="1517414412931" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>#1517413835602</inState>
    <outState>#1517414374046</outState>
  </transitions>
  <transitions id="1517414413132" name="MISSING_NAME" comment="" msg="">
    <preCondition id="1517414413939" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>#1517414374046</inState>
    <outState>#1517414385931</outState>
  </transitions>
  <transitions id="1517415414091" name="MISSING_NAME" comment="" msg="">
    <preCondition id="1517415415438" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>#1517414385931</inState>
    <outState>#1517414888265</outState>
  </transitions>
  <transitions id="1517415421538" name="MISSING_NAME" comment="" msg="">
    <preCondition id="1517415422853" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>#1517414888265</inState>
    <outState>#1517414896747</outState>
  </transitions>
  <transitions id="1517415427753" name="MISSING_NAME" comment="" msg="">
    <preCondition id="1517415428866" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>#1517414896747</inState>
    <outState>#1517415180269</outState>
  </transitions>
  <transitions id="1517415435160" name="MISSING_NAME" comment="" msg="">
    <preCondition id="1517415436553" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>#1517415180269</inState>
    <outState>#1517415192926</outState>
  </transitions>
  <entryPoints id="1517050590882" name="MISSING_NAME" comment="" successRequired="false" minCardinality="0" maxCardinality="2147483647">
    <task>../../../Misc/taskrepository.tsk#1225112227903</task>
    <state>#1517050590881</state>
  </entryPoints>
</alica:Plan>
