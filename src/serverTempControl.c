#include <wiringPi.h>
#include <stdio.h>
#include <netinet/in.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/socket.h>
#include <unistd.h>
#include <time.h>
#define PORT 8080


int Artuga1 = 5, Artuga2 = 6, Artuga3 = 13, Artuga4 = 19, Artuga5 = 26;

int tempArtuga1 = 1, tempArtuga2 = 7, tempArtuga3 = 8, tempArtuga4 = 25, tempArtuga5 = 24;

int main(int argc, char const* argv[]){

    // wiringPiSetupGpio();
    // pinMode(Artuga1, 1);//OUTPUT
    // pinMode(Artuga2, 1);
    // pinMode(Artuga3, 1);
    // pinMode(Artuga4, 1);
    // pinMode(Artuga5, 1);

    // pinMode(tempArtuga1, 0);//INPUT
    // pinMode(tempArtuga2, 0);
    // pinMode(tempArtuga3, 0);
    // pinMode(tempArtuga4, 0);
    // pinMode(tempArtuga5, 0);

    int server_fd, new_socket, valread;
    struct sockaddr_in address;
    int opt = 1;
    int addrlen = sizeof(address);
    char buffer[1024] = { 0 };

    // Creating socket file descriptor
    if ((server_fd = socket(AF_INET, SOCK_STREAM, 0))
        == 0) {
        perror("socket failed");
        exit(EXIT_FAILURE);
    }
    // Forcefully attaching socket to the port 8080
    if (setsockopt(server_fd, SOL_SOCKET,
                   SO_REUSEADDR | SO_REUSEPORT, &opt,
                   sizeof(opt))) {
        perror("setsockopt");
        exit(EXIT_FAILURE);
    }
    address.sin_family = AF_INET;
    address.sin_addr.s_addr = INADDR_ANY;
    address.sin_port = htons(PORT);
 
    // Forcefully attaching socket to the port 8080
    if (bind(server_fd, (struct sockaddr*)&address,
             sizeof(address))
        < 0) {
        perror("bind failed");
        exit(EXIT_FAILURE);
    }
    if (listen(server_fd, 3) < 0) {
        perror("listen");
        exit(EXIT_FAILURE);
    }
    if ((new_socket
         = accept(server_fd, (struct sockaddr*)&address,
                  (socklen_t*)&addrlen))
        < 0) {
        perror("accept");
        exit(EXIT_FAILURE);
    }


    int flag = 0;

    while (1)
    {

        valread = read(new_socket, buffer, 1024);
        buffer[valread] = '\0';
        printf("buffer %s\n",buffer);

        if((strcmp(buffer,"on")==0 )&& (flag ==0)){
            printf("Temp ON\n");
            flag = 1;
            // digitalWrite(Artuga1,digitalRead(tempArtuga1));
            // digitalWrite(Artuga2,digitalRead(tempArtuga2));
            // digitalWrite(Artuga3,digitalRead(tempArtuga3));
            // digitalWrite(Artuga4,digitalRead(tempArtuga4));
            // digitalWrite(Artuga5,digitalRead(tempArtuga5));
            
        }

        if((strcmp(buffer,"off")==0) && (flag == 1)){
            printf("Temp OFF\n");
            flag = 0;
            // digitalWrite(Artuga1,0);
            // digitalWrite(Artuga2,0);
            // digitalWrite(Artuga3,0);
            // digitalWrite(Artuga4,0);
            // digitalWrite(Artuga5,0);

        }
        delay(1);
        if ((new_socket
         = accept(server_fd, (struct sockaddr*)&address,
                  (socklen_t*)&addrlen))
        < 0)
            break;

    }
    
    return 0;

}