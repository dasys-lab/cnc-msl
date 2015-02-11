function show_images()

clear;

path = pwd;

i=0;
while( true )
    
    file = [path,'/image_',int2str(i),'.csv'];
    if( exist(file, 'file') == 2 )
        M = dlmread(file,';',0,0);
        figure(i+1,'Name','Image from optical flow sensor's);
        imshow(M,[4 252]);
        i = i+ 1;
    end
    
    pause(1);
end


