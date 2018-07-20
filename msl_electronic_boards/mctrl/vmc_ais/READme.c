/*
Änderungen von Version 94g ->

 0_95a

main.c:	
         // Selects the function to handle the command if applicable
 //@@@@@       sel_proceed_cmd();
		sel_proceed_allcmd();


		// Load command from RS232 and stores it into the commandbuffer
 //@@@@@       cmdb_load_command(_CMDB_CH_ALL_, _CMDB_ONE_);
        cmdb_load_command(_CMDB_CH_ALL_, _CMDB_ALL_);

        // Sends response from commandbuffer to RS232
//@@@@@       cmdb_send_command(_CMDB_ONE_);
       cmdb_send_command(_CMDB_ALL_);




*/
