function read_ball_motion()
% Auslesen der Motion über den seriellen Port und Anzeige der Ballbewegung.

    logfile_folder = 'Logfiles/';   % Ordner in dem die Logfiles gespeichert werden

    % Globale Variable zum Abbruch der Funktionsausführung
    global run_flag;
    run_flag = 1;
   
    % Figure zur Vektoranzeige erstellen und anpassen
    figure_handle_1 = figure(1);
    clf('reset');
    set(figure_handle_1,'Name','Ball Motion','KeyPressFcn',@stopper,'Position',[50,65,550,546]);
    image = imshow('fussball.jpg','Border','tight');
    warning off last;    % Warnung ausschalten, dass das Bild zu groß ist
    set(0,'CurrentFigure',figure_handle_1);    % Figure in Vordergrund holen
    % Mittelpunkt hinzufügen
    dot_size = 0.02;
    annotation('ellipse',[0.5-dot_size/2,0.5-dot_size/2,dot_size,dot_size],'FaceColor','red');
    % Koordinatensystem hinzufügen
    annotation('arrow',[0.05,0.15],[0.05,0.05]);
    annotation('arrow',[0.05,0.05],[0.05,0.15]);
    annotation('textbox',[0.13,0.015,0.04,0.04],'String','x','FontWeight','bold','LineStyle','none');
    annotation('textbox',[0.015,0.13,0.04,0.04],'String','y','FontWeight','bold','LineStyle','none');
    drawnow;
    
     % Figure zur Balken-Diagramm-Anzeige erstellen und anpassen
%     figure_handle_2 = figure(2);
%     set(figure_handle_2,'Name','Ball Motion - Bar Diagram','KeyPressFcn',@stopper,'Position',[650,65,560,546]);

    % Prüfen ob serielle Ports verfügbar sind
    if ~isempty(instrfind)
        fclose(instrfind);
    end

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
    
    % Variablen zu Normierung des Motion-Vektors
    delta_max = 128;        % Maximum des Motion-Delta-Wertes
    vector_max = 0.5;       % Maximale Länge des Vektors im Figure
    vector_scale = 0.9;     % Skalierung des Vektors
    vector_norm = delta_max/(vector_max);%*vector_scale);

    % Daten einlesen
    while (run_flag)
        newline = fgetl(ser);   % Zeile einlesen
        disp(newline)           % Zeile ausgeben
        
        % Zeile zerlegen (Muster: 'Motion;Delta_x;Delta_y;SQUAL;...')
        motion_data = str2num(newline);
        
        % Falsch empfangene Zeilen ignorieren 
        if (length(motion_data) ~= 4)
            continue;
        end
        
        % Balken-Diagramm erstellen
%         set(0,'CurrentFigure',figure_handle_2);    % Figure in Vordergrund holen
%          clf(figure_handle_2);
%         bar_handle = bar([2 2 2;4 4 4],[motion_data(2:4)';1 1 1],0.3);
%         ylim([-140 180]);
%         xlim([1.3 2.7]);
%         legend(bar_handle,['Delta X: ',num2str(motion_data(2))],['Delta Y: ',num2str(motion_data(3))],['SQUAL: ',num2str(motion_data(4))],'Location','SouthOutside');
%         set(gca,'YGrid','on',...
%                 'YTick',[-140:20:180],...
%                 'XTickLabel',{'','',''});
%         hold on;
%         plot([0 0 0;4 4 4],[128 128 128;128 128 128],'LineStyle','--','Color','red');       % rote Begrenzungslinie max
%         plot([0 0 0;4 4 4],[-128 -128 -128;-128 -128 -128],'LineStyle','--','Color','red'); % rote Begrenzungslinie min
%         plot([0 0 0;4 4 4],[169 169 169;169 169 169],'LineStyle',':','Color','red'); % Begrenzungslinie SQUAL
%         drawnow;
            
        motion = [motion_data(2) motion_data(3)];
        % Motion in Figure darstellen
        if (max(motion == 0))
             arrow_xy = [0,0];
             arrow_position = [1,1];    % Start Position des Vektors [x,y] (außerhalb des figures)
        else
             arrow_xy = [motion(1)/vector_norm,motion(2)/vector_norm];    % Normierung des Vektors
             arrow_position = [0.5,0.5];    % Start Position des Vektors [x,y]
        end
        % Pfeil beim ersten Aufruf erstellen
        if ~(exist('arrow_handle','var'))
             arrow_handle = annotation(figure_handle_1,'arrow',[arrow_position(1),arrow_xy(1)+arrow_position(1)],[arrow_position(2),arrow_xy(2)+arrow_position(2)],'Color','red','LineWidth',2);
             drawnow;
        else
            set(arrow_handle,'Position',[arrow_position(1),arrow_position(2),arrow_xy(1),arrow_xy(2)]);
             drawnow;
        end
    end
end