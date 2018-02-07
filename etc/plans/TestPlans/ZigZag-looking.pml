<?xml version="1.0" encoding="ASCII"?>
<alica:Plan xmi:version="2.0" xmlns:xmi="http://www.omg.org/XMI" xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance" xmlns:alica="http:///de.uni_kassel.vs.cn" id="1517415924695" name="ZigZag-looking" comment="" masterPlan="false" utilityFunction="" utilityThreshold="0.1" destinationPath="" priority="0.0" minCardinality="0" maxCardinality="2147483647">
  <states id="1517416042696" name="NewState" comment="" entryPoint="1517416042697">
    <outTransitions>#1517418040032</outTransitions>
  </states>
  <states id="1517416696784" name="Zi" comment="">
    <plans xsi:type="alica:BehaviourConfiguration">../GenericBehaviours/DriveAndAlignToPoint.beh#1517416779403</plans>
    <inTransitions>#1517418040032</inTransitions>
    <outTransitions>#1517418041334</outTransitions>
  </states>
  <states id="1517416701665" name="Za" comment="">
    <plans xsi:type="alica:BehaviourConfiguration">../GenericBehaviours/DriveAndAlignToPoint.beh#1517416856727</plans>
    <inTransitions>#1517418041334</inTransitions>
    <outTransitions>#1517418042142</outTransitions>
  </states>
  <states id="1517416705659" name="Zig" comment="">
    <plans xsi:type="alica:BehaviourConfiguration">../GenericBehaviours/DriveAndAlignToPoint.beh#1517416964916</plans>
    <inTransitions>#1517418042142</inTransitions>
    <outTransitions>#1517418043041</outTransitions>
  </states>
  <states id="1517416709251" name="Zag" comment="">
    <plans xsi:type="alica:BehaviourConfiguration">../GenericBehaviours/DriveAndAlignToPoint.beh#1517416991102</plans>
    <inTransitions>#1517418043041</inTransitions>
    <outTransitions>#1517418044061</outTransitions>
  </states>
  <states id="1517416712742" name="Ziggi" comment="">
    <plans xsi:type="alica:BehaviourConfiguration">../GenericBehaviours/DriveAndAlignToPoint.beh#1517417012255</plans>
    <inTransitions>#1517418044061</inTransitions>
    <outTransitions>#1517418045594</outTransitions>
  </states>
  <states id="1517417015545" name="Zaggi" comment="">
    <plans xsi:type="alica:BehaviourConfiguration">../GenericBehaviours/DriveAndAlignToPoint.beh#1517417110968</plans>
    <inTransitions>#1517418045594</inTransitions>
    <outTransitions>#1517418046519</outTransitions>
  </states>
  <states id="1517417019794" name="Zi1" comment="">
    <plans xsi:type="alica:BehaviourConfiguration">../GenericBehaviours/DriveAndAlignToPoint.beh#1517417780816</plans>
    <inTransitions>#1517418046519</inTransitions>
    <outTransitions>#1517418047444</outTransitions>
  </states>
  <states id="1517417737805" name="Za1" comment="">
    <plans xsi:type="alica:BehaviourConfiguration">../GenericBehaviours/DriveAndAlignToPoint.beh#1517417835320</plans>
    <inTransitions>#1517418047444</inTransitions>
    <outTransitions>#1517419593802</outTransitions>
  </states>
  <states id="1517419486798" name="Zi1" comment="">
    <plans xsi:type="alica:BehaviourConfiguration">../GenericBehaviours/DriveAndAlignToPoint.beh#1517417780816</plans>
    <inTransitions>#1517419593802</inTransitions>
    <outTransitions>#1517419601683</outTransitions>
  </states>
  <states id="1517419516960" name="Zaggi" comment="">
    <plans xsi:type="alica:BehaviourConfiguration">../GenericBehaviours/DriveAndAlignToPoint.beh#1517417110968</plans>
    <inTransitions>#1517419601683</inTransitions>
    <outTransitions>#1517419598555</outTransitions>
  </states>
  <states id="1517419530561" name="Ziggi" comment="">
    <plans xsi:type="alica:BehaviourConfiguration">../GenericBehaviours/DriveAndAlignToPoint.beh#1517417012255</plans>
    <inTransitions>#1517419598555</inTransitions>
    <outTransitions>#1517419602999</outTransitions>
  </states>
  <states id="1517419548563" name="Zag" comment="">
    <plans xsi:type="alica:BehaviourConfiguration">../GenericBehaviours/DriveAndAlignToPoint.beh#1517416991102</plans>
    <inTransitions>#1517419602999</inTransitions>
    <outTransitions>#1517419604348</outTransitions>
  </states>
  <states id="1517419558004" name="Zig" comment="">
    <plans xsi:type="alica:BehaviourConfiguration">../GenericBehaviours/DriveAndAlignToPoint.beh#1517416964916</plans>
    <inTransitions>#1517419604348</inTransitions>
    <outTransitions>#1517419605477</outTransitions>
  </states>
  <states id="1517419566037" name="Za" comment="">
    <plans xsi:type="alica:BehaviourConfiguration">../GenericBehaviours/DriveAndAlignToPoint.beh#1517416856727</plans>
    <inTransitions>#1517419605477</inTransitions>
    <inTransitions>#1517419614928</inTransitions>
    <outTransitions>#1517419606734</outTransitions>
  </states>
  <states id="1517419578382" name="Zi" comment="">
    <plans xsi:type="alica:BehaviourConfiguration">../GenericBehaviours/DriveAndAlignToPoint.beh#1517416779403</plans>
    <inTransitions>#1517419606734</inTransitions>
    <outTransitions>#1517419607579</outTransitions>
    <outTransitions>#1517419614928</outTransitions>
  </states>
  <states id="1517419585143" name="NewState" comment="">
    <inTransitions>#1517419607579</inTransitions>
    <outTransitions>#1517419610438</outTransitions>
  </states>
  <states xsi:type="alica:SuccessState" id="1517419589232" name="NewSuccessState" comment="">
    <inTransitions>#1517419610438</inTransitions>
  </states>
  <transitions id="1517418040032" name="MISSING_NAME" comment="" msg="">
    <preCondition id="1517418041205" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>#1517416042696</inState>
    <outState>#1517416696784</outState>
  </transitions>
  <transitions id="1517418041334" name="MISSING_NAME" comment="" msg="">
    <preCondition id="1517418042021" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>#1517416696784</inState>
    <outState>#1517416701665</outState>
  </transitions>
  <transitions id="1517418042142" name="MISSING_NAME" comment="" msg="">
    <preCondition id="1517418042897" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>#1517416701665</inState>
    <outState>#1517416705659</outState>
  </transitions>
  <transitions id="1517418043041" name="MISSING_NAME" comment="" msg="">
    <preCondition id="1517418043909" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>#1517416705659</inState>
    <outState>#1517416709251</outState>
  </transitions>
  <transitions id="1517418044061" name="MISSING_NAME" comment="" msg="">
    <preCondition id="1517418045434" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>#1517416709251</inState>
    <outState>#1517416712742</outState>
  </transitions>
  <transitions id="1517418045594" name="MISSING_NAME" comment="" msg="">
    <preCondition id="1517418046382" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>#1517416712742</inState>
    <outState>#1517417015545</outState>
  </transitions>
  <transitions id="1517418046519" name="MISSING_NAME" comment="" msg="">
    <preCondition id="1517418047348" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>#1517417015545</inState>
    <outState>#1517417019794</outState>
  </transitions>
  <transitions id="1517418047444" name="MISSING_NAME" comment="" msg="">
    <preCondition id="1517418048518" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>#1517417019794</inState>
    <outState>#1517417737805</outState>
  </transitions>
  <transitions id="1517419593802" name="MISSING_NAME" comment="" msg="">
    <preCondition id="1517419598274" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>#1517417737805</inState>
    <outState>#1517419486798</outState>
  </transitions>
  <transitions id="1517419598555" name="MISSING_NAME" comment="" msg="">
    <preCondition id="1517419601443" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>#1517419516960</inState>
    <outState>#1517419530561</outState>
  </transitions>
  <transitions id="1517419601683" name="MISSING_NAME" comment="" msg="">
    <preCondition id="1517419602823" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>#1517419486798</inState>
    <outState>#1517419516960</outState>
  </transitions>
  <transitions id="1517419602999" name="MISSING_NAME" comment="" msg="">
    <preCondition id="1517419604187" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>#1517419530561</inState>
    <outState>#1517419548563</outState>
  </transitions>
  <transitions id="1517419604348" name="MISSING_NAME" comment="" msg="">
    <preCondition id="1517419605219" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>#1517419548563</inState>
    <outState>#1517419558004</outState>
  </transitions>
  <transitions id="1517419605477" name="MISSING_NAME" comment="" msg="">
    <preCondition id="1517419606486" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>#1517419558004</inState>
    <outState>#1517419566037</outState>
  </transitions>
  <transitions id="1517419606734" name="MISSING_NAME" comment="" msg="">
    <preCondition id="1517419607427" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>#1517419566037</inState>
    <outState>#1517419578382</outState>
  </transitions>
  <transitions id="1517419607579" name="MISSING_NAME" comment="" msg="">
    <preCondition id="1517419608860" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>#1517419578382</inState>
    <outState>#1517419585143</outState>
  </transitions>
  <transitions id="1517419610438" name="MISSING_NAME" comment="" msg="">
    <preCondition id="1517419612655" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>#1517419585143</inState>
    <outState>#1517419589232</outState>
  </transitions>
  <transitions id="1517419614928" name="MISSING_NAME" comment="" msg="">
    <preCondition id="1517419617136" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>#1517419578382</inState>
    <outState>#1517419566037</outState>
  </transitions>
  <entryPoints id="1517416042697" name="MISSING_NAME" comment="" successRequired="true" minCardinality="0" maxCardinality="2147483647">
    <task>../../Misc/taskrepository.tsk#1225112227903</task>
    <state>#1517416042696</state>
  </entryPoints>
</alica:Plan>
