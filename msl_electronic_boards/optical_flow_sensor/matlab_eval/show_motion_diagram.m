function show_motion_diagram(save_data)
% Auslesen der Motion und Anzeige der Deltas in einem Diagram.
% Über den Parameter save_data=true kann festgelegt werden, ob die gelesenen 
% Daten gespeichert werden sollen.
    
    logfile_folder = 'Logfiles/';   % Ordner in dem die Logfiles gespeichert werden

    % Globale Variable zum Abbruch der Funktionsausführung
    global run_flag;
    run_flag = 1;
 
    % Default: Bilddatei nicht speichern
    if(nargin == 0)
        save_data = false;
    end

    % Prüfen ob serielle Ports verfügbar sind
    if ~isempty(instrfind)
        fclose(instrfind);
    end
    
    % Figure erstellen und anpassen
    figure_handle = figure(1);
    clf('reset');
    set(figure_handle,'Name','Ball Motion','KeyPressFcn',@stopper);

    % Serial-Objekt erzeugen und Parameter einstellen
    global ser;
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
    
    % Datei zum speichern der Daten öffnen
    if( save_data )
            folder = sprintf([logfile_folder,'%s'],datestr(now,30)); % Ordnername nach Zeit
            mkdir(folder);  % Ordner erstellen
            file = [folder,'/data.csv'];
            fileID = fopen(file,'a+');  % Datei öffnen
    end

    counter = 0;
    % Daten einlesen
    while (run_flag)              % Endlosschleife
        newline = fgetl(ser);   % Zeile einlesen
        disp(newline)           % Zeile ausgeben
        
        % Zeile zerlegen (Muster: 'Motion;Delta_x;Delta_y;SQUAL;...')
        motion_data = str2num(newline);

        % Falsch empfangene Zeilen ignorieren 
        if (length(motion_data) ~= 4)
            flushinput(ser);    % Seriellen Puffer leeren
            continue;
        end
        
        if(counter == 0)
                % Matrizen für Daten erzeugen
                time=0;
                delta_x=motion_data(2);
                delta_y=motion_data(3);
                squal=motion_data(4);
        else
            time = [time counter];
            delta_x=[delta_x motion_data(2)];
            delta_y=[delta_y motion_data(3)];
            squal=[squal motion_data(4)];
        end
        
        % Daten Plotten
        plot(time,delta_x,time,delta_y,time,squal);
        grid on;
        drawnow;
        
        if( save_data )         % Daten in Datei schreiben
            fprintf(fileID,newline);   % Zeile in Datei schreiben (anhängen)
            fprintf(fileID,'\n\r');       % neue Zeile in Datei
        end
        counter = counter + 1;
    end
    % Verbindung schließen
    fclose(ser);
    % Achsenbeschriftung und Legende in Plot zeichnen
    ylabel('Value');
    xlabel('Data Counter');
    legend('Delta X','Delta Y','SQUAL','Location','BestOutside');

    if( save_data )
        fclose(fileID);             % Daten-Datei schließen
        print(figure_handle,'-depsc',[folder,'/plot.eps']);    %Plot speichern
    end
end

