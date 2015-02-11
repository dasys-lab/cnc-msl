#include <unistd.h>
#include <cstdio>
#include <stdlib.h>
#include <sys/ioctl.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <sys/un.h>
#include <netdb.h>
#include <signal.h>
#include <iostream>

using namespace std;

int main( int argc, char *argv[] ) {
        struct sockaddr_in serv_adr;
cout << "1" << endl;

        if ( argc != 3 ) {                   // Check cmd line for host name
                cerr << "usage: " << argv[0] << " server port_#" << endl;
                return 1;
        }
        struct hostent *host = gethostbyname(argv[1]);
cout << "2" << endl;
        if (host == (struct hostent *) NULL ) {
                perror("gethostbyname ");
                return 2;
        }

        memset(&serv_adr, 0, sizeof( serv_adr));       // Clear structure
        serv_adr.sin_family = AF_INET;                 // Set address type
        memcpy(&serv_adr.sin_addr, host->h_addr, host->h_length);
        serv_adr.sin_port   = htons( atoi(argv[2]) );
cout << "3" << endl;
        int orig_sock = socket(PF_INET, SOCK_STREAM, 0);
cout << "4" << endl;
        if (connect( orig_sock,(struct sockaddr *)&serv_adr, sizeof(serv_adr)) < 0) {
                perror("connect error");
                return 4;
        }
        //
cout << "5" << endl;
        write(orig_sock, "test", 4);
        char buf[32];
cout << "6" << endl;
        read(orig_sock, buf, 4);
cout << "->" << buf << "<-" << endl;
cout << "7" << endl;
        //
        close(orig_sock);
        return 0;
} 
