MW=load("/media/D818-8ABC/MW.csv")
[MWM, MWG, MWGG, MWS, MWSG]=generateMesh(MW,80)
[MWGH, MWGV]=GHF(MWM, MWS, 30000)
printHGLUT(MWGV(:,5))
