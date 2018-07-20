#!/bin/sh
grep -rl 'alica:Result' ./ | xargs sed -i 's/alica:Result/alica:PostCondition/g'

grep -rl 'planDesigner\/ui\/extensionmodel' ./ | xargs sed -i 's/ui\/extensionmodel/pmlextension\/uiextensionmodel/g'

grep -rl 'de.uni_kassel.cn' ./ | xargs sed -i 's/de.uni_kassel.cn/de.uni_kassel.vs.cn/g'

grep -rl '<preCondition' ./ | xargs sed -i 's/<preCondition \(.*\) conditionString/<preCondition \1 comment/g'

grep -rl '<preCondition' ./ | xargs sed -i 's/<preCondition \(.*\) comment=\"\" /<preCondition \1 conditionString=\"\" /g'

grep -rl '<result' ./ | xargs sed -i 's/<result/<postCondition/g'

find . -name "*.pmlex" | xargs sed -i 's/<uiextensionmodel:PmlUiExtensionMap\(.*\)xmi:version="2.0"/<uiextensionmodel:PmlUiExtensionMap\1/g'

