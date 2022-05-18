// Client side C/C++ program to demonstrate Socket
// programming
#include <arpa/inet.h>
#include <stdio.h>
#include <string.h>
#include <sys/socket.h>
#include <unistd.h>
#include <stdlib.h>
#include <time.h>
#include <netdb.h>
#define PORT 8080
 
int main(int argc, char const* argv[])
{
    int sock = 0, valread;
    struct sockaddr_in serv_addr;
    char* hello = "Liga";
    char buffer[1024] = { 0 };
    if ((sock = socket(AF_INET, SOCK_STREAM, 0)) < 0) {
        printf("\n Socket creation error \n");
        return -1;
    }
 
    serv_addr.sin_family = AF_INET;
    serv_addr.sin_port = htons(PORT);
 
    //serv_addr.sin_addr.s_addr = inet_addr("10.42.0.76");
    // Convert IPv4 and IPv6 addresses from text to binary
    // form

    struct hostent *hp;
    hp = gethostbyname("10.42.0.76"); 
    if (hp == 0) { 
	printf("asdasdasdasd"); 
	return -1; 
    } 
  
  bcopy(hp->h_addr, &serv_addr.sin_addr, hp->h_length); 
  serv_addr.sin_family = AF_INET; 
  serv_addr.sin_port = htons(8080);  ; 

  int n = connect(sock, (struct sockaddr *)&serv_addr,
	      sizeof (serv_addr));
  
  
  if(n == -1)
  {
      perror("E803 connect returned:");
      close(sock);
      sock = -1;
      return -1;
  }
  

    
    printf("Sending message \n");

    send(sock, argv[1], strlen(argv[1]), 0);
    printf("Message Sent \n");

    return 0;
}