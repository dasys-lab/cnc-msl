function stopper(~,evnt)
global run_flag ser;
if evnt.Character == 'x'
    run_flag = 0;
end
if (isvalid(ser) && evnt.Character == 'f')
    flushinput(ser);
end
clear run_flag;
end