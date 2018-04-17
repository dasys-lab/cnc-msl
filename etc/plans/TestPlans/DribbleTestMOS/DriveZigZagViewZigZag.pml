<?xml version="1.0" encoding="ASCII"?>
<alica:Plan xmi:version="2.0" xmlns:xmi="http://www.omg.org/XMI" xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance" xmlns:alica="http:///de.uni_kassel.vs.cn" id="1518623186077" name="DriveZigZagViewZigZag" comment="" masterPlan="false" utilityFunction="" utilityThreshold="0.1" destinationPath="Plans/TestPlans/DribbleTestMOS" priority="0.0" minCardinality="0" maxCardinality="2147483647">
  <states id="1518623225899" name="Zig1" comment="" entryPoint="1518623225900">
    <plans xsi:type="alica:BehaviourConfiguration">../../GenericBehaviours/DriveAndAlignToPoint.beh#1517415399331</plans>
    <outTransitions>#1518625925018</outTransitions>
  </states>
  <states id="1518623354324" name="Zag1" comment="">
    <plans xsi:type="alica:BehaviourConfiguration">../../GenericBehaviours/DriveAndAlignToPoint.beh#1517052566683</plans>
    <inTransitions>#1518625925018</inTransitions>
    <outTransitions>#1518625926691</outTransitions>
  </states>
  <states id="1518623356568" name="Zig2" comment="">
    <plans xsi:type="alica:BehaviourConfiguration">../../GenericBehaviours/DriveAndAlignToPoint.beh#1517052704566</plans>
    <inTransitions>#1518625926691</inTransitions>
    <outTransitions>#1518625928160</outTransitions>
  </states>
  <states id="1518623359185" name="Zag2" comment="">
    <plans xsi:type="alica:BehaviourConfiguration">../../GenericBehaviours/DriveAndAlignToPoint.beh#1517054090420</plans>
    <inTransitions>#1518625928160</inTransitions>
    <outTransitions>#1518625929687</outTransitions>
  </states>
  <states id="1518623360811" name="Zig3" comment="">
    <plans xsi:type="alica:BehaviourConfiguration">../../GenericBehaviours/DriveAndAlignToPoint.beh#1517054165498</plans>
    <inTransitions>#1518625929687</inTransitions>
    <outTransitions>#1518625931911</outTransitions>
  </states>
  <states id="1518623422054" name="Zag3" comment="">
    <plans xsi:type="alica:BehaviourConfiguration">../../GenericBehaviours/DriveAndAlignToPoint.beh#1517054241072</plans>
    <inTransitions>#1518625931911</inTransitions>
    <outTransitions>#1518625933475</outTransitions>
  </states>
  <states id="1518623595252" name="Zig4" comment="">
    <plans xsi:type="alica:BehaviourConfiguration">../../GenericBehaviours/DriveAndAlignToPoint.beh#1517054288108</plans>
    <inTransitions>#1518625933475</inTransitions>
    <outTransitions>#1518625935166</outTransitions>
  </states>
  <states id="1518623598950" name="Zag4" comment="">
    <plans xsi:type="alica:BehaviourConfiguration">../../GenericBehaviours/DriveAndAlignToPoint.beh#1517054365259</plans>
    <inTransitions>#1518625935166</inTransitions>
    <outTransitions>#1518625936475</outTransitions>
  </states>
  <states id="1518623603896" name="Zig5" comment="">
    <plans xsi:type="alica:BehaviourConfiguration">../../GenericBehaviours/DriveAndAlignToPoint.beh#1517054380022</plans>
    <inTransitions>#1518625936475</inTransitions>
    <outTransitions>#1518625938490</outTransitions>
  </states>
  <states id="1518623607938" name="Zag5" comment="">
    <plans xsi:type="alica:BehaviourConfiguration">../../GenericBehaviours/DriveAndAlignToPoint.beh#1517054559436</plans>
    <inTransitions>#1518625938490</inTransitions>
    <outTransitions>#1518625939758</outTransitions>
  </states>
  <states id="1518625001282" name="Zig6" comment="">
    <plans xsi:type="alica:BehaviourConfiguration">../../GenericBehaviours/DriveAndAlignToPoint.beh#1517054583522</plans>
    <inTransitions>#1518625939758</inTransitions>
    <outTransitions>#1518625941142</outTransitions>
  </states>
  <states id="1518625006119" name="Zag6" comment="">
    <plans xsi:type="alica:BehaviourConfiguration">../../GenericBehaviours/DriveAndAlignToPoint.beh#1517054756760</plans>
    <inTransitions>#1518625941142</inTransitions>
    <outTransitions>#1518625943009</outTransitions>
  </states>
  <states id="1518625011128" name="Turn" comment="">
    <plans xsi:type="alica:BehaviourConfiguration">../../GenericBehaviours/DriveAndAlignToPoint.beh#1517054780047</plans>
    <inTransitions>#1518625943009</inTransitions>
    <outTransitions>#1518625944620</outTransitions>
  </states>
  <states id="1518625014361" name="Zag7" comment="">
    <plans xsi:type="alica:BehaviourConfiguration">../../GenericBehaviours/DriveAndAlignToPoint.beh#1517413343977</plans>
    <inTransitions>#1518625944620</inTransitions>
    <outTransitions>#1518625959741</outTransitions>
  </states>
  <states id="1518625129996" name="Zig7" comment="">
    <plans xsi:type="alica:BehaviourConfiguration">../../GenericBehaviours/DriveAndAlignToPoint.beh#1517413374873</plans>
    <inTransitions>#1518625959741</inTransitions>
    <outTransitions>#1518625963172</outTransitions>
  </states>
  <states id="1518625221164" name="Zag8" comment="">
    <plans xsi:type="alica:BehaviourConfiguration">../../GenericBehaviours/DriveAndAlignToPoint.beh#1517413717087</plans>
    <inTransitions>#1518625963172</inTransitions>
    <outTransitions>#1518625964419</outTransitions>
  </states>
  <states id="1518625223133" name="Zig8" comment="">
    <plans xsi:type="alica:BehaviourConfiguration">../../GenericBehaviours/DriveAndAlignToPoint.beh#1517413743789</plans>
    <inTransitions>#1518625964419</inTransitions>
    <outTransitions>#1518625965744</outTransitions>
  </states>
  <states id="1518625245016" name="Zag9" comment="">
    <plans xsi:type="alica:BehaviourConfiguration">../../GenericBehaviours/DriveAndAlignToPoint.beh#1517413986109</plans>
    <inTransitions>#1518625965744</inTransitions>
    <outTransitions>#1518625978947</outTransitions>
  </states>
  <states id="1518625246754" name="Zig9" comment="">
    <plans xsi:type="alica:BehaviourConfiguration">../../GenericBehaviours/DriveAndAlignToPoint.beh#1517414007376</plans>
    <inTransitions>#1518625978947</inTransitions>
    <outTransitions>#1518625980704</outTransitions>
  </states>
  <states id="1518625266418" name="Zag10" comment="">
    <plans xsi:type="alica:BehaviourConfiguration">../../GenericBehaviours/DriveAndAlignToPoint.beh#1517414454147</plans>
    <inTransitions>#1518625980704</inTransitions>
    <outTransitions>#1518625985861</outTransitions>
  </states>
  <states id="1518625276571" name="Zig10" comment="">
    <plans xsi:type="alica:BehaviourConfiguration">../../GenericBehaviours/DriveAndAlignToPoint.beh#1517414498820</plans>
    <inTransitions>#1518625985861</inTransitions>
    <outTransitions>#1518625987819</outTransitions>
  </states>
  <states id="1518625287036" name="Zag11" comment="">
    <plans xsi:type="alica:BehaviourConfiguration">../../GenericBehaviours/DriveAndAlignToPoint.beh#1517415111859</plans>
    <inTransitions>#1518625987819</inTransitions>
    <outTransitions>#1518625989203</outTransitions>
  </states>
  <states id="1518625290581" name="Zig11" comment="">
    <plans xsi:type="alica:BehaviourConfiguration">../../GenericBehaviours/DriveAndAlignToPoint.beh#1517415170220</plans>
    <inTransitions>#1518625989203</inTransitions>
    <outTransitions>#1518625990575</outTransitions>
  </states>
  <states id="1518625302958" name="Zag12" comment="">
    <plans xsi:type="alica:BehaviourConfiguration">../../GenericBehaviours/DriveAndAlignToPoint.beh#1517415329054</plans>
    <inTransitions>#1518625990575</inTransitions>
    <outTransitions>#1518625992939</outTransitions>
  </states>
  <states id="1518625307527" name="Zig12" comment="">
    <plans xsi:type="alica:BehaviourConfiguration">../../GenericBehaviours/DriveAndAlignToPoint.beh#1517415399331</plans>
    <inTransitions>#1518625992939</inTransitions>
    <outTransitions>#1518869291908</outTransitions>
  </states>
  <states xsi:type="alica:SuccessState" id="1518869281982" name="NewSuccessState" comment="">
    <inTransitions>#1518869291908</inTransitions>
  </states>
  <transitions id="1518625925018" name="MISSING_NAME" comment="" msg="">
    <preCondition id="1518625926563" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>#1518623225899</inState>
    <outState>#1518623354324</outState>
  </transitions>
  <transitions id="1518625926691" name="MISSING_NAME" comment="" msg="">
    <preCondition id="1518625927967" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>#1518623354324</inState>
    <outState>#1518623356568</outState>
  </transitions>
  <transitions id="1518625928160" name="MISSING_NAME" comment="" msg="">
    <preCondition id="1518625929607" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>#1518623356568</inState>
    <outState>#1518623359185</outState>
  </transitions>
  <transitions id="1518625929687" name="MISSING_NAME" comment="" msg="">
    <preCondition id="1518625931710" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>#1518623359185</inState>
    <outState>#1518623360811</outState>
  </transitions>
  <transitions id="1518625931911" name="MISSING_NAME" comment="" msg="">
    <preCondition id="1518625933379" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>#1518623360811</inState>
    <outState>#1518623422054</outState>
  </transitions>
  <transitions id="1518625933475" name="MISSING_NAME" comment="" msg="">
    <preCondition id="1518625934916" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>#1518623422054</inState>
    <outState>#1518623595252</outState>
  </transitions>
  <transitions id="1518625935166" name="MISSING_NAME" comment="" msg="">
    <preCondition id="1518625936267" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>#1518623595252</inState>
    <outState>#1518623598950</outState>
  </transitions>
  <transitions id="1518625936475" name="MISSING_NAME" comment="" msg="">
    <preCondition id="1518625938386" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>#1518623598950</inState>
    <outState>#1518623603896</outState>
  </transitions>
  <transitions id="1518625938490" name="MISSING_NAME" comment="" msg="">
    <preCondition id="1518625939622" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>#1518623603896</inState>
    <outState>#1518623607938</outState>
  </transitions>
  <transitions id="1518625939758" name="MISSING_NAME" comment="" msg="">
    <preCondition id="1518625941030" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>#1518623607938</inState>
    <outState>#1518625001282</outState>
  </transitions>
  <transitions id="1518625941142" name="MISSING_NAME" comment="" msg="">
    <preCondition id="1518625942849" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>#1518625001282</inState>
    <outState>#1518625006119</outState>
  </transitions>
  <transitions id="1518625943009" name="MISSING_NAME" comment="" msg="">
    <preCondition id="1518625944363" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>#1518625006119</inState>
    <outState>#1518625011128</outState>
  </transitions>
  <transitions id="1518625944620" name="MISSING_NAME" comment="" msg="">
    <preCondition id="1518625947022" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>#1518625011128</inState>
    <outState>#1518625014361</outState>
  </transitions>
  <transitions id="1518625959741" name="MISSING_NAME" comment="" msg="">
    <preCondition id="1518625962322" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>#1518625014361</inState>
    <outState>#1518625129996</outState>
  </transitions>
  <transitions id="1518625963172" name="MISSING_NAME" comment="" msg="">
    <preCondition id="1518625964162" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>#1518625129996</inState>
    <outState>#1518625221164</outState>
  </transitions>
  <transitions id="1518625964419" name="MISSING_NAME" comment="" msg="">
    <preCondition id="1518625965631" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>#1518625221164</inState>
    <outState>#1518625223133</outState>
  </transitions>
  <transitions id="1518625965744" name="MISSING_NAME" comment="" msg="">
    <preCondition id="1518625978714" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>#1518625223133</inState>
    <outState>#1518625245016</outState>
  </transitions>
  <transitions id="1518625978947" name="MISSING_NAME" comment="" msg="">
    <preCondition id="1518625979903" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>#1518625245016</inState>
    <outState>#1518625246754</outState>
  </transitions>
  <transitions id="1518625980704" name="MISSING_NAME" comment="" msg="">
    <preCondition id="1518625985243" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>#1518625246754</inState>
    <outState>#1518625266418</outState>
  </transitions>
  <transitions id="1518625985861" name="MISSING_NAME" comment="" msg="">
    <preCondition id="1518625986946" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>#1518625266418</inState>
    <outState>#1518625276571</outState>
  </transitions>
  <transitions id="1518625987819" name="MISSING_NAME" comment="" msg="">
    <preCondition id="1518625988962" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>#1518625276571</inState>
    <outState>#1518625287036</outState>
  </transitions>
  <transitions id="1518625989203" name="MISSING_NAME" comment="" msg="">
    <preCondition id="1518625990399" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>#1518625287036</inState>
    <outState>#1518625290581</outState>
  </transitions>
  <transitions id="1518625990575" name="MISSING_NAME" comment="" msg="">
    <preCondition id="1518625992739" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>#1518625290581</inState>
    <outState>#1518625302958</outState>
  </transitions>
  <transitions id="1518625992939" name="MISSING_NAME" comment="" msg="">
    <preCondition id="1518625993923" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>#1518625302958</inState>
    <outState>#1518625307527</outState>
  </transitions>
  <transitions id="1518869291908" name="MISSING_NAME" comment="" msg="">
    <preCondition id="1518869293795" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>#1518625307527</inState>
    <outState>#1518869281982</outState>
  </transitions>
  <entryPoints id="1518623225900" name="MISSING_NAME" comment="" successRequired="true" minCardinality="0" maxCardinality="2147483647">
    <task>../../../Misc/taskrepository.tsk#1225112227903</task>
    <state>#1518623225899</state>
  </entryPoints>
</alica:Plan>
