function show_images_from_dir(save_img)
% Gibt alle Bilder eines Verzeichnisses aus. Wird der Paramter true
% übergeben, werden die Bilder als Bilddatei gespeichert.
    
    dir_name = uigetdir(pwd,'Verzeichnis wählen');
    
    % Wurde der Button Abbrechen gedrückt?
    if (dir_name == 0)
        disp('Funktionsaufruf abgebrochen.')
        return;
    end
    
    % Default: Bilddatei nicht speichern
    if(nargin == 0)
        save_img = false;
    end

    %file = [dir_name,'/image_0.csv'];  % alte Dateinamen
    file = [dir_name,'/image_000.csv']; % neue Dateinamen
    img_counter=0;
    while( exist(file, 'file') == 2 )
        M = dlmread(file,';',0,0);
        figure_handle = figure(img_counter+1);
        clf('reset');
        image = imshow(M,[4 252]);
        drawnow;
        
        if (save_img)
            % Ergebnise: pdf - Pixelbild, aber ganze Seite, eps - klein,
            % aber verschwommen, png - Pixelbild, aber ganzes figure im
            % bild
            %saveas(image,[file,'.eps']);
            %imwrite(uint8(M),[file,'.tif']);   %save as tif
            %print(figure_handle,'-depsc',[file,'.eps']);   % save as eps
            print(figure_handle,'-dpng',[file,'.png']);   % save
        end

        img_counter = img_counter+ 1;
        %file = [dir_name,'/image_',int2str(img_counter),'.csv'];
        file = [dir_name,'/image_',sprintf('%03u',img_counter),'.csv'];
        
    end
end