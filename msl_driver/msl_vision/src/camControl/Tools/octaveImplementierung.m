function [MeshMatrix, GainMatrix, GainMatrixGlatt, ShutterMatrix, ShutterMatrixGlatt]=generateMesh(MessDaten, ndFor)
%Anzeigbare Matrix (Mash-Befehl) erzeugen
for (i=1:rows(MessDaten)/ndFor); 
	for (j=1:ndFor); 
		MeshMatrix(i,j)=MessDaten((i-1)*ndFor+j,3); 
	end; 
end;

%Gainmatrix erzeugen
for (i=2:rows(MeshMatrix)-1); 
	for (j=1:columns(MeshMatrix)); 
		GainMatrix(i-1,j)=(MeshMatrix(i+1, j)-MeshMatrix(i-1, j))/2; 
	end; 
end;

%Geglättete Gainmatrix erzeugen
for (i=1:rows(MeshMatrix)-2); 
	for (j=2:columns(MeshMatrix)-1); 
		GainMatrixGlatt(i,j)=MeshMatrix(i+2, j-1)-MeshMatrix(i, j-1)+MeshMatrix(i+2, j)-MeshMatrix(i, j)+MeshMatrix(i+2, j+1)-MeshMatrix(i, j+1); 
	end; 
end;

%Shuttermatrix erzeugen
for (i=1:rows(MeshMatrix)); 
	for (j=2:columns(MeshMatrix)-1); 
		ShutterMatrix(i,j-1)=(MeshMatrix(i, j+1)-MeshMatrix(i, j-1))/2; 
	end; 
end;

%Geglättete Shuttermatrix erzeugen
for (i=2:rows(MeshMatrix)-1); 
	for (j=1:columns(MeshMatrix)-2); 
		ShutterMatrixGlatt(i,j)=MeshMatrix(i-1, j+2)-MeshMatrix(i-1, j)+MeshMatrix(i, j+2)-MeshMatrix(i, j)+MeshMatrix(i+1, j+2)-MeshMatrix(i+1, j); 
	end; 
end;

endfunction



function [GainHellMatrix, GainHellVektor]=GHF(MeshMatrix, GainMatrix, maxVal)
maxHell=0;

for (i=1:rows(GainMatrix));
	for (j=1:columns(GainMatrix));
		if (GainMatrix(i,j)>maxHell);
			maxHell=GainMatrix(i,j);
		end;	
	end;
end;

GainHellMatrix=zeros(255,11+maxHell);
for (i=1:rows(GainMatrix));
	for (j=1:columns(GainMatrix));
		if (GainMatrix(i,j)>0);
			if (round(GainMatrix(i,j))==GainMatrix(i,j));
				GainHellMatrix(MeshMatrix(i,j)+1,GainMatrix(i,j)+10)=GainHellMatrix(MeshMatrix(i,j)+1,GainMatrix(i,j)+10)+1;
			else;
				GainHellMatrix(MeshMatrix(i,j)+1,GainMatrix(i,j)-0.5+10)=GainHellMatrix(MeshMatrix(i,j)+1,GainMatrix(i,j)-0.5+10)+1;
				GainHellMatrix(MeshMatrix(i,j)+1,GainMatrix(i,j)+0.5+10)=GainHellMatrix(MeshMatrix(i,j)+1,GainMatrix(i,j)+0.5+10)+1;
			end;
		end;
	end;
end;


GainHellVektor=zeros(256,4);
for (i=2:rows(GainHellMatrix));
	GainHellVektor(i,1)=i;
	for (j=1:columns(GainHellMatrix));
		GainHellVektor(i, 2)=GainHellVektor(i, 2)+GainHellMatrix(i, j)*j;
		GainHellVektor(i, 3)=GainHellVektor(i, 3)+GainHellMatrix(i, j);			
	end;
	if (GainHellVektor(i, 3)>0);
		GainHellVektor(i, 4)=GainHellVektor(i, 2)/GainHellVektor(i, 3);
		GainHellVektor(i,4)-=10;
	end;
end;

for (i=2:rows(GainHellMatrix));
	if (GainHellVektor(i,3)==0);
		nae=0;
		while (i+nae<rows(GainHellMatrix)-2 && GainHellVektor(i+nae,3)==0);
			nae+=1;
		end;
		i
		nae
		GainHellVektor(i+nae+1,4)
		GainHellVektor(i,4)=(GainHellVektor(i-1,4)*nae+GainHellVektor(i+nae+1,4))/(nae+1);
		%GainHellVektor(i,4)=GainHellVektor(i-1,4);
	end;

end;

GainHellVektor(1,5)=0.5;
GainHellVektor(2,5)=0.5;
GainHellVektor(3,5)=0.5;
GainHellVektor(4,5)=0.5;
GainHellVektor(253,5)=0.5;
GainHellVektor(254,5)=0.5;
GainHellVektor(255,5)=0.5;
GainHellVektor(256,5)=0.5;

for (i=5:252);
	GainHellVektor(i,5)=(GainHellVektor(i-4,4)+GainHellVektor(i-3,4)+GainHellVektor(i-2,4)+GainHellVektor(i-1,4)+GainHellVektor(i,4)+GainHellVektor(i+1,4)+GainHellVektor(i+2,4)+GainHellVektor(i+3,4)+GainHellVektor(i+4,4))/9;
	if (GainHellVektor(i,5)<0.1);
		GainHellVektor(i,5)=0.1;
	end;
end;

for (i=1:rows(GainHellMatrix));
	for (j=1:columns(GainHellMatrix));
		GainHellMatrix(i,j)=min([GainHellMatrix(i,j),maxVal]);
	end;
end;

for (i=1:rows(GainHellMatrix));
	for (j=1:columns(GainHellMatrix));
		GainHellMatrix(i,j)=min([GainHellMatrix(i,j),maxVal]);
	end;
end;

endfunction


function printHGLUT(Daten)
for (i=1:rows(Daten));
	printf("\t\tHGLUT%i = %f\n",i-1,Daten(i));
end;
endfunction;




