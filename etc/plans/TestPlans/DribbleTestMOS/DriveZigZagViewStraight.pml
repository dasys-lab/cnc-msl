<?xml version="1.0" encoding="ASCII"?>
<alica:Plan xmi:version="2.0" xmlns:xmi="http://www.omg.org/XMI" xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance" xmlns:alica="http:///de.uni_kassel.vs.cn" id="1518623233217" name="DriveZigZagViewStraight" comment="" masterPlan="false" utilityFunction="" utilityThreshold="0.1" destinationPath="Plans/TestPlans/DribbleTestMOS" priority="0.0" minCardinality="0" maxCardinality="2147483647">
  <states id="1518623253780" name="Zig1" comment="" entryPoint="1518623253781">
    <plans xsi:type="alica:BehaviourConfiguration">../../GenericBehaviours/DriveAndAlignToPoint.beh#1518630243941</plans>
    <outTransitions>#1518862112736</outTransitions>
  </states>
  <states id="1518626788701" name="Zag1" comment="">
    <plans xsi:type="alica:BehaviourConfiguration">../../GenericBehaviours/DriveAndAlignToPoint.beh#1518630243942</plans>
    <inTransitions>#1518862112736</inTransitions>
    <outTransitions>#1518862114684</outTransitions>
  </states>
  <states id="1518626804312" name="Zig2" comment="">
    <plans xsi:type="alica:BehaviourConfiguration">../../GenericBehaviours/DriveAndAlignToPoint.beh#1518630243943</plans>
    <inTransitions>#1518862114684</inTransitions>
    <outTransitions>#1518862115908</outTransitions>
  </states>
  <states id="1518626806747" name="Zag2" comment="">
    <plans xsi:type="alica:BehaviourConfiguration">../../GenericBehaviours/DriveAndAlignToPoint.beh#1518630243951</plans>
    <inTransitions>#1518862115908</inTransitions>
    <outTransitions>#1518862117076</outTransitions>
  </states>
  <states id="1518626813522" name="Zig3" comment="">
    <plans xsi:type="alica:BehaviourConfiguration">../../GenericBehaviours/DriveAndAlignToPoint.beh#1518630243961</plans>
    <inTransitions>#1518862117076</inTransitions>
    <outTransitions>#1518862118444</outTransitions>
  </states>
  <states id="1518626818294" name="Zag3" comment="">
    <plans xsi:type="alica:BehaviourConfiguration">../../GenericBehaviours/DriveAndAlignToPoint.beh#1518630243971</plans>
    <inTransitions>#1518862118444</inTransitions>
    <outTransitions>#1518862119772</outTransitions>
  </states>
  <states id="1518626900749" name="Zig4" comment="">
    <plans xsi:type="alica:BehaviourConfiguration">../../GenericBehaviours/DriveAndAlignToPoint.beh#1518630244941</plans>
    <inTransitions>#1518862119772</inTransitions>
    <outTransitions>#1518862121816</outTransitions>
  </states>
  <states id="1518626902606" name="Zag4" comment="">
    <plans xsi:type="alica:BehaviourConfiguration">../../GenericBehaviours/DriveAndAlignToPoint.beh#1518630245941</plans>
    <inTransitions>#1518862121816</inTransitions>
    <outTransitions>#1518862124036</outTransitions>
  </states>
  <states id="1518626904168" name="Zig5" comment="">
    <plans xsi:type="alica:BehaviourConfiguration">../../GenericBehaviours/DriveAndAlignToPoint.beh#1518630245942</plans>
    <inTransitions>#1518862124036</inTransitions>
    <outTransitions>#1518862127047</outTransitions>
  </states>
  <states id="1518626906208" name="Zag5" comment="">
    <plans xsi:type="alica:BehaviourConfiguration">../../GenericBehaviours/DriveAndAlignToPoint.beh#1518630245943</plans>
    <inTransitions>#1518862127047</inTransitions>
    <outTransitions>#1518862129245</outTransitions>
  </states>
  <states id="1518626915328" name="Zig6" comment="">
    <plans xsi:type="alica:BehaviourConfiguration">../../GenericBehaviours/DriveAndAlignToPoint.beh#1518630246041</plans>
    <inTransitions>#1518862129245</inTransitions>
    <outTransitions>#1518862131169</outTransitions>
  </states>
  <states id="1518626921067" name="Zag6" comment="">
    <plans xsi:type="alica:BehaviourConfiguration">../../GenericBehaviours/DriveAndAlignToPoint.beh#1518630248941</plans>
    <inTransitions>#1518862131169</inTransitions>
    <outTransitions>#1518862132987</outTransitions>
  </states>
  <states id="1518627895634" name="Turn" comment="">
    <plans xsi:type="alica:BehaviourConfiguration">../../GenericBehaviours/DriveAndAlignToPoint.beh#1518632911294</plans>
    <inTransitions>#1518862132987</inTransitions>
    <outTransitions>#1518862139276</outTransitions>
  </states>
  <states id="1518627905490" name="Zag7" comment="">
    <plans xsi:type="alica:BehaviourConfiguration">../../GenericBehaviours/DriveAndAlignToPoint.beh#1518630248941</plans>
    <inTransitions>#1518862139276</inTransitions>
    <outTransitions>#1518862141035</outTransitions>
  </states>
  <states id="1518627908250" name="Zig7" comment="">
    <plans xsi:type="alica:BehaviourConfiguration">../../GenericBehaviours/DriveAndAlignToPoint.beh#1518630246041</plans>
    <inTransitions>#1518862141035</inTransitions>
    <outTransitions>#1518862142752</outTransitions>
  </states>
  <states id="1518627910387" name="Zag8" comment="">
    <plans xsi:type="alica:BehaviourConfiguration">../../GenericBehaviours/DriveAndAlignToPoint.beh#1518630245943</plans>
    <inTransitions>#1518862142752</inTransitions>
    <outTransitions>#1518862144196</outTransitions>
  </states>
  <states id="1518627912340" name="Zig8" comment="">
    <plans xsi:type="alica:BehaviourConfiguration">../../GenericBehaviours/DriveAndAlignToPoint.beh#1518630245942</plans>
    <inTransitions>#1518862144196</inTransitions>
    <outTransitions>#1518862150499</outTransitions>
  </states>
  <states id="1518627914142" name="Zag9" comment="">
    <plans xsi:type="alica:BehaviourConfiguration">../../GenericBehaviours/DriveAndAlignToPoint.beh#1518630245941</plans>
    <inTransitions>#1518862150499</inTransitions>
    <outTransitions>#1518862152090</outTransitions>
  </states>
  <states id="1518627927737" name="Zig9" comment="">
    <plans xsi:type="alica:BehaviourConfiguration">../../GenericBehaviours/DriveAndAlignToPoint.beh#1518630244941</plans>
    <inTransitions>#1518862152090</inTransitions>
    <outTransitions>#1518862153421</outTransitions>
  </states>
  <states id="1518627930880" name="Zag10" comment="">
    <plans xsi:type="alica:BehaviourConfiguration">../../GenericBehaviours/DriveAndAlignToPoint.beh#1518630243971</plans>
    <inTransitions>#1518862153421</inTransitions>
    <outTransitions>#1518862155229</outTransitions>
  </states>
  <states id="1518627932536" name="Zig10" comment="">
    <plans xsi:type="alica:BehaviourConfiguration">../../GenericBehaviours/DriveAndAlignToPoint.beh#1518630243961</plans>
    <inTransitions>#1518862155229</inTransitions>
    <outTransitions>#1518862156436</outTransitions>
  </states>
  <states id="1518627933858" name="Zag11" comment="">
    <plans xsi:type="alica:BehaviourConfiguration">../../GenericBehaviours/DriveAndAlignToPoint.beh#1518630243951</plans>
    <inTransitions>#1518862156436</inTransitions>
    <outTransitions>#1518862157824</outTransitions>
  </states>
  <states id="1518627935733" name="Zig11" comment="">
    <plans xsi:type="alica:BehaviourConfiguration">../../GenericBehaviours/DriveAndAlignToPoint.beh#1518630243943</plans>
    <inTransitions>#1518862157824</inTransitions>
    <outTransitions>#1518862159753</outTransitions>
  </states>
  <states id="1518627937452" name="Zag12" comment="">
    <plans xsi:type="alica:BehaviourConfiguration">../../GenericBehaviours/DriveAndAlignToPoint.beh#1518630243942</plans>
    <inTransitions>#1518862159753</inTransitions>
    <outTransitions>#1518862163150</outTransitions>
  </states>
  <states id="1518627939373" name="Zig12" comment="">
    <plans xsi:type="alica:BehaviourConfiguration">../../GenericBehaviours/DriveAndAlignToPoint.beh#1518630243941</plans>
    <inTransitions>#1518862163150</inTransitions>
    <outTransitions>#1518869321445</outTransitions>
  </states>
  <states xsi:type="alica:SuccessState" id="1518869314037" name="NewSuccessState" comment="">
    <inTransitions>#1518869321445</inTransitions>
  </states>
  <transitions id="1518862112736" name="MISSING_NAME" comment="" msg="">
    <preCondition id="1518862114467" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>#1518623253780</inState>
    <outState>#1518626788701</outState>
  </transitions>
  <transitions id="1518862114684" name="MISSING_NAME" comment="" msg="">
    <preCondition id="1518862115739" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>#1518626788701</inState>
    <outState>#1518626804312</outState>
  </transitions>
  <transitions id="1518862115908" name="MISSING_NAME" comment="" msg="">
    <preCondition id="1518862116884" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>#1518626804312</inState>
    <outState>#1518626806747</outState>
  </transitions>
  <transitions id="1518862117076" name="MISSING_NAME" comment="" msg="">
    <preCondition id="1518862118075" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>#1518626806747</inState>
    <outState>#1518626813522</outState>
  </transitions>
  <transitions id="1518862118444" name="MISSING_NAME" comment="" msg="">
    <preCondition id="1518862119588" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>#1518626813522</inState>
    <outState>#1518626818294</outState>
  </transitions>
  <transitions id="1518862119772" name="MISSING_NAME" comment="" msg="">
    <preCondition id="1518862121592" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>#1518626818294</inState>
    <outState>#1518626900749</outState>
  </transitions>
  <transitions id="1518862121816" name="MISSING_NAME" comment="" msg="">
    <preCondition id="1518862123708" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>#1518626900749</inState>
    <outState>#1518626902606</outState>
  </transitions>
  <transitions id="1518862124036" name="MISSING_NAME" comment="" msg="">
    <preCondition id="1518862126807" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>#1518626902606</inState>
    <outState>#1518626904168</outState>
  </transitions>
  <transitions id="1518862127047" name="MISSING_NAME" comment="" msg="">
    <preCondition id="1518862129037" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>#1518626904168</inState>
    <outState>#1518626906208</outState>
  </transitions>
  <transitions id="1518862129245" name="MISSING_NAME" comment="" msg="">
    <preCondition id="1518862130944" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>#1518626906208</inState>
    <outState>#1518626915328</outState>
  </transitions>
  <transitions id="1518862131169" name="MISSING_NAME" comment="" msg="">
    <preCondition id="1518862132763" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>#1518626915328</inState>
    <outState>#1518626921067</outState>
  </transitions>
  <transitions id="1518862132987" name="MISSING_NAME" comment="" msg="">
    <preCondition id="1518862135489" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>#1518626921067</inState>
    <outState>#1518627895634</outState>
  </transitions>
  <transitions id="1518862139276" name="MISSING_NAME" comment="" msg="">
    <preCondition id="1518862140627" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>#1518627895634</inState>
    <outState>#1518627905490</outState>
  </transitions>
  <transitions id="1518862141035" name="MISSING_NAME" comment="" msg="">
    <preCondition id="1518862142552" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>#1518627905490</inState>
    <outState>#1518627908250</outState>
  </transitions>
  <transitions id="1518862142752" name="MISSING_NAME" comment="" msg="">
    <preCondition id="1518862143987" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>#1518627908250</inState>
    <outState>#1518627910387</outState>
  </transitions>
  <transitions id="1518862144196" name="MISSING_NAME" comment="" msg="">
    <preCondition id="1518862148362" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>#1518627910387</inState>
    <outState>#1518627912340</outState>
  </transitions>
  <transitions id="1518862150499" name="MISSING_NAME" comment="" msg="">
    <preCondition id="1518862151625" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>#1518627912340</inState>
    <outState>#1518627914142</outState>
  </transitions>
  <transitions id="1518862152090" name="MISSING_NAME" comment="" msg="">
    <preCondition id="1518862153164" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>#1518627914142</inState>
    <outState>#1518627927737</outState>
  </transitions>
  <transitions id="1518862153421" name="MISSING_NAME" comment="" msg="">
    <preCondition id="1518862154788" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>#1518627927737</inState>
    <outState>#1518627930880</outState>
  </transitions>
  <transitions id="1518862155229" name="MISSING_NAME" comment="" msg="">
    <preCondition id="1518862156091" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>#1518627930880</inState>
    <outState>#1518627932536</outState>
  </transitions>
  <transitions id="1518862156436" name="MISSING_NAME" comment="" msg="">
    <preCondition id="1518862157639" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>#1518627932536</inState>
    <outState>#1518627933858</outState>
  </transitions>
  <transitions id="1518862157824" name="MISSING_NAME" comment="" msg="">
    <preCondition id="1518862159584" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>#1518627933858</inState>
    <outState>#1518627935733</outState>
  </transitions>
  <transitions id="1518862159753" name="MISSING_NAME" comment="" msg="">
    <preCondition id="1518862162949" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>#1518627935733</inState>
    <outState>#1518627937452</outState>
  </transitions>
  <transitions id="1518862163150" name="MISSING_NAME" comment="" msg="">
    <preCondition id="1518862164476" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>#1518627937452</inState>
    <outState>#1518627939373</outState>
  </transitions>
  <transitions id="1518869321445" name="MISSING_NAME" comment="" msg="">
    <preCondition id="1518869323662" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>#1518627939373</inState>
    <outState>#1518869314037</outState>
  </transitions>
  <entryPoints id="1518623253781" name="MISSING_NAME" comment="" successRequired="true" minCardinality="0" maxCardinality="2147483647">
    <task>../../../Misc/taskrepository.tsk#1225112227903</task>
    <state>#1518623253780</state>
  </entryPoints>
</alica:Plan>
