<?xml version="1.0" encoding="ASCII"?>
<alica:Plan xmi:version="2.0" xmlns:xmi="http://www.omg.org/XMI" xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance" xmlns:alica="http:///de.uni_kassel.vs.cn" id="1434045709191" name="AttackPlay" comment="" masterPlan="false" utilityFunction="" utilityThreshold="0.1" destinationPath="Plans/Attack" priority="0.0" minCardinality="2" maxCardinality="2147483647">
  <conditions xsi:type="alica:RuntimeCondition" id="1434112519736" name="NewRuntimeCondition" comment="haveBall" conditionString="" pluginName="DefaultPlugin"/>
  <states id="1434045709193" name="Attack" comment="" entryPoint="1434045709194">
    <plans xsi:type="alica:Plan">StandardAttack.pml#1434046634656</plans>
  </states>
  <states id="1434045868018" name="AttackSupport" comment="" entryPoint="1434045719840">
    <plans xsi:type="alica:Plan">AttackSupportPlan.pml#1434046705214</plans>
  </states>
  <states id="1434045870617" name="Defend" comment="" entryPoint="1434045723977"/>
  <states id="1434112762535" name="OfferForPass" comment="" entryPoint="1434112675755">
    <plans xsi:type="alica:Plan">RunFree.pml#1434115664325</plans>
  </states>
  <entryPoints id="1434045709194" name="Attack" comment="" successRequired="false" minCardinality="1" maxCardinality="1">
    <task>../../Misc/taskrepository.tsk#1222613952469</task>
    <state>#1434045709193</state>
  </entryPoints>
  <entryPoints id="1434045719840" name="AttackSupport" comment="" successRequired="false" minCardinality="0" maxCardinality="2147483647">
    <task>../../Misc/taskrepository.tsk#1225115536468</task>
    <state>#1434045868018</state>
  </entryPoints>
  <entryPoints id="1434045723977" name="Defend" comment="" successRequired="false" minCardinality="1" maxCardinality="2147483647">
    <task>../../Misc/taskrepository.tsk#1225115406909</task>
    <state>#1434045870617</state>
  </entryPoints>
  <entryPoints id="1434112675755" name="MISSING_NAME" comment="" successRequired="false" minCardinality="0" maxCardinality="2147483647">
    <task>../../Misc/taskrepository.tsk#1434112683820</task>
    <state>#1434112762535</state>
  </entryPoints>
</alica:Plan>
