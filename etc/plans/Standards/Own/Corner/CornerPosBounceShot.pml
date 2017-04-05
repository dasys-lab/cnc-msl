<?xml version="1.0" encoding="ASCII"?>
<alica:Plan xmi:version="2.0" xmlns:xmi="http://www.omg.org/XMI" xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance" xmlns:alica="http:///de.uni_kassel.vs.cn" id="1459361999064" name="CornerPosBounceShot" comment="" destinationPath="Plans/Standards/Own/Corner" priority="0.0" minCardinality="2" maxCardinality="4" masterPlan="false" utilityFunction="" utilityThreshold="0.1">
  <states id="1459362020729" name="AlignPasser" comment="" entryPoint="1459362020730">
    <plans xsi:type="alica:Behaviour">Behaviours/ShovelSelect.beh#1434199834892</plans>
    <plans xsi:type="alica:Behaviour">Standards/Own/Corner/BounceShotAlignPasser.beh#1459354938404</plans>
  </states>
  <states id="1459362636939" name="AlignWall" comment="" entryPoint="1459362605142">
    <plans xsi:type="alica:Behaviour">Standards/Own/Corner/BounceShotAlignWall.beh#1459355002202</plans>
  </states>
  <states id="1459362640122" name="DefenderWall" comment="" entryPoint="1459362606949">
    <plans xsi:type="alica:Behaviour">Standards/Own/Corner/StandardDefendPos.beh#1459355042700</plans>
  </states>
  <entryPoints id="1459362020730" name="ExecuteStandard" comment="" successRequired="false" minCardinality="1" maxCardinality="1">
    <task>taskrepository.tsk#1439997010902</task>
    <state>Standards/Own/Corner/CornerPosBounceShot.pml#1459362020729</state>
  </entryPoints>
  <entryPoints id="1459362605142" name="ReceiveStandard" comment="" successRequired="false" minCardinality="1" maxCardinality="1">
    <task>taskrepository.tsk#1439997023446</task>
    <state>Standards/Own/Corner/CornerPosBounceShot.pml#1459362636939</state>
  </entryPoints>
  <entryPoints id="1459362606949" name="Defend" comment="" successRequired="false" minCardinality="0" maxCardinality="2">
    <task>taskrepository.tsk#1225115406909</task>
    <state>Standards/Own/Corner/CornerPosBounceShot.pml#1459362640122</state>
  </entryPoints>
</alica:Plan>
