function read_serial_images(save_img)
% Liest Bilder über den seriellen Port ein, speichert sie in einem
% separaten Ordner und gibt sie aus. Wird der Paramter true
% übergeben, werden die Bilder als Bilddatei gespeichert.

    logfile_folder = 'Logfiles/';   % Ordner in dem die Logfiles gespeichert werden

    % Default: Bilddatei nicht speichern
    if(nargin == 0)
        save_img = false;
    end
    
    % Prüfen ob serielle Ports verfügbar sind
    if ~isempty(instrfind)
        fclose(instrfind);
    end

    % Serial-Objekt erzeugen und Parameter einstellen
    ser = serial('COM3');
    disp('Serialobjekt erzeugt');
    set(ser,'BaudRate',9600);
    set(ser,'DataBits',8);
    set(ser,'Parity','none');
    set(ser,'StopBits',1);
    set(ser,'ReadAsyncMode','continuous');
    set(ser,'Terminator','CR/LF');

    % Verbindung herstellen
    fopen(ser);
    disp('Serialobjekt geöffnet')

    % Globale Variable zum Abbruch der Funktionsausführung
    global run_flag;
    run_flag = 1;
    
    % Ordner für Bilder erstellen
    img_folder = sprintf([logfile_folder,'%s'],datestr(now,30)); % Ordnername nach Zeit
    mkdir(img_folder);

    img_counter = 0;
    
    % Daten einlesen
    while (run_flag)
        newline = fgetl(ser);   % Zeile einlesen
        disp([newline 'Image:' int2str(img_counter)])           % Zeile ausgeben

        if(newline(1)=='-')     % neues Bild?
            file = [img_folder,'/image_',sprintf('%03u',img_counter),'.csv']; %dateiname
            fileID = fopen(file,'wt');        % Datei öffnen
            
            % Daten in Datei schreiben
            for i=1:1:30
                nextline = fgetl(ser);        % nächste Zeile abfragen
                fprintf(fileID,nextline);     % Zeile in Datei schreiben
                fprintf(fileID,'\n');         % neue Zeile in Datei
            end
            fclose(fileID);                   % Datei schließen

            % Bild ausgeben
            %M = (dlmread(file,';',0,0));   % csv-Datei einlesen
            M = (dlmread(file,';',0,0))';   % csv-Datei einlesen und Bild drehen
            figure_handle = figure(1);
            clf;
            set(figure_handle,'Name','Image from optical flow sensor','KeyPressFcn',@stopper);
            %set(0,'CurrentFigure',figure_handle);   % Figure in Vordergrund holen
            image = imshow(M,[4 252]);
            drawnow;
            
            % Bild speichern
            if (save_img == true)
                saveas(image,[file,'.eps']);
                %imwrite(uint8(M),[file,'.tif']);   %save as tif
                %print(figure_handle,'-depsc',[file,'.eps']);   % save as eps
            end
            img_counter = img_counter + 1;
        end
    end
    % Verbindung schließen
    fclose(ser);
end