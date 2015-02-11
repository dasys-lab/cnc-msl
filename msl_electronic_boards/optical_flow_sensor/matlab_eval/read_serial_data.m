function read_serial_data(save_data)
% Liest Daten über den seriellen Port ein und gibt sie aus. Über den
% Parameter save_data=true kann festgelegt werden, ob die gelesenen Daten
% gespeichert werden sollen.
    
    logfile_folder = 'Logfiles/';   % Ordner in dem die Logfiles gespeichert werden
 
    % Default: Bilddatei nicht speichern
    if(nargin == 0)
        save_data = false;
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
    
    % Daten einlesen
    while ( true )              % Endlosschleife
        newline = fgetl(ser);   % Zeile einlesen
        disp(newline)           % Zeile ausgeben
        
        if( save_data )         % Daten in Datei schreiben
            file = sprintf([logfile_folder,'/%s.txt'],datestr(now,30)); % Ordnername nach Zeit
            fileID = fopen(file,'a+');  % Datei öffnen
            fprintf(fileID,newline);   % Zeile in Datei schreiben (anhängen)
            fprintf(fileID,'\n');       % neue Zeile in Datei
            fclose(fileID);             % Datei schließen
        end
    end
    % Verbindung schließen
    fclose(ser);
end