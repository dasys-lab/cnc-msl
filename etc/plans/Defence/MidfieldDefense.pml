<?xml version="1.0" encoding="ASCII"?>
<alica:Plan xmi:version="2.0" xmlns:xmi="http://www.omg.org/XMI" xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance" xmlns:alica="http:///de.uni_kassel.vs.cn" id="1458033329973" name="MidfieldDefense" comment="" masterPlan="false" utilityFunction="" utilityThreshold="0.1" destinationPath="Plans/Defence" priority="0.0" minCardinality="0" maxCardinality="2147483647">
  <states id="1458033358325" name="MidFieldBlock" comment="" entryPoint="1458033358326">
    <plans xsi:type="alica:Plan">MidfieldBlock.pml#1458033620834</plans>
    <outTransitions>#1458033410354</outTransitions>
  </states>
  <states id="1458033385978" name="ReleaseMid" comment="">
    <plans xsi:type="alica:BehaviourConfiguration">ReleaseMid.beh#1458033497042</plans>
    <inTransitions>#1458033410354</inTransitions>
    <inTransitions>#1458033412731</inTransitions>
    <outTransitions>#1458033411608</outTransitions>
  </states>
  <states id="1458033395158" name="ReleaseOwnHalf" comment="">
    <plans xsi:type="alica:Plan">ReleaseOwnHalf.pml#1458033644590</plans>
    <inTransitions>#1458033411608</inTransitions>
    <outTransitions>#1458033412731</outTransitions>
  </states>
  <transitions id="1458033410354" name="MISSING_NAME" comment="MidFieldBlock2ReleaseMid: Blocking failed or is successfull &amp; Attacker is in opp half" msg="">
    <preCondition id="1458033411271" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>#1458033358325</inState>
    <outState>#1458033385978</outState>
  </transitions>
  <transitions id="1458033411608" name="MISSING_NAME" comment="ReleaseMid2ReleaseOwnHalf:" msg="">
    <preCondition id="1458033412464" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>#1458033385978</inState>
    <outState>#1458033395158</outState>
  </transitions>
  <transitions id="1458033412731" name="MISSING_NAME" comment="ReleaseOwnHalf2RleaseMid" msg="">
    <preCondition id="1458033413418" name="MISSING_NAME" comment="" conditionString="" pluginName="DefaultPlugin" enabled="true"/>
    <inState>#1458033395158</inState>
    <outState>#1458033385978</outState>
  </transitions>
  <entryPoints id="1458033358326" name="MISSING_NAME" comment="" successRequired="false" minCardinality="0" maxCardinality="2147483647">
    <task>../../Misc/taskrepository.tsk#1225112227903</task>
    <state>#1458033358325</state>
  </entryPoints>
</alica:Plan>
