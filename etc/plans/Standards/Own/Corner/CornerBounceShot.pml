<?xml version="1.0" encoding="ASCII"?>
<alica:Plan xmi:version="2.0" xmlns:xmi="http://www.omg.org/XMI" xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance" xmlns:alica="http:///de.uni_kassel.vs.cn" id="1459361887233" name="CornerBounceShot" comment="" masterPlan="false" utilityFunction="" utilityThreshold="0.1" destinationPath="Plans/Standards/Own/Corner" priority="0.0" minCardinality="0" maxCardinality="2147483647">
  <states id="1459361887234" name="CornerPos" comment="" entryPoint="1459361887235">
    <plans xsi:type="alica:Plan">CornerPosBounceShot.pml#1459361999064</plans>
    <outTransitions>#1459361911154</outTransitions>
  </states>
  <states id="1459361895479" name="DoCorner" comment="">
    <plans xsi:type="alica:Plan">CornerExecBounceShot.pml#1459362028865</plans>
    <inTransitions>#1459361911154</inTransitions>
    <outTransitions>#1459361912344</outTransitions>
    <outTransitions>#1459361914595</outTransitions>
  </states>
  <states xsi:type="alica:SuccessState" id="1459361899313" name="NewSuccessState" comment="">
    <inTransitions>#1459361912344</inTransitions>
  </states>
  <states xsi:type="alica:FailureState" id="1459361900873" name="NewFailureState" comment="">
    <inTransitions>#1459361914595</inTransitions>
  </states>
  <transitions id="1459361911154" name="MISSING_NAME" comment="CornerPos2DoCorner" msg="">
    <preCondition id="1459361912141" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>#1459361887234</inState>
    <outState>#1459361895479</outState>
  </transitions>
  <transitions id="1459361912344" name="MISSING_NAME" comment="DoCorner2Succ" msg="">
    <preCondition id="1459361914252" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>#1459361895479</inState>
    <outState>#1459361899313</outState>
  </transitions>
  <transitions id="1459361914595" name="MISSING_NAME" comment="DoCorner2Fail" msg="">
    <preCondition id="1459361916700" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>#1459361895479</inState>
    <outState>#1459361900873</outState>
  </transitions>
  <entryPoints id="1459361887235" name="MISSING_NAME" comment="" successRequired="false" minCardinality="0" maxCardinality="2147483647">
    <task>../../../../Misc/taskrepository.tsk#1225112227903</task>
    <state>#1459361887234</state>
  </entryPoints>
</alica:Plan>
