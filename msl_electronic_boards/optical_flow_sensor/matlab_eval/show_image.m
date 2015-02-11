function show_image(save_img)
% Zeigt ein Bild des Sensors an aus einer csv-Datei an. Wird der Paramter true
% übergeben, wird das Bild als Bilddatei gespeichert.
    
    [filename,dirname,filterindex] = uigetfile('*.csv','Datei wählen');
    file = [dirname,'/',filename];
    
    % Wurde der Button Abbrechen gedrückt?
    if (filterindex == 0)
        disp('Funktionsaufruf abgebrochen.')
        return;
    end
    
    % Default: Bilddatei nicht speichern
    if(nargin == 0)
        save_img = false;
    end

    
    M = dlmread(file,';',0,0);
    scrsz = get(0,'ScreenSize');
    figure_handle = figure(1);
    clf('reset');
    set(figure_handle,'Name','Image from optical flow sensor');    % Name figure
    image = imshow(M,[4 252]);
    %set(figure_handler,'Position',[scrsz(3)/4 scrsz(4)/4 scrsz(3)/2 scrsz(4)/2]);   % Resize figure
    
    if ( save_img )
        saveas(image,[file,'.eps']);
        %imwrite(uint8(M),[file,'.tif']);   %save as tif
        %print(figure_handle,'-depsc',[file,'.eps']);   % save as eps
    end
end