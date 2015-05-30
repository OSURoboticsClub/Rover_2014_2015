#include "detector_client.h"
#include <unistd.h>
using namespace std;
using namespace cv;


int processImage(Mat &img){
    Mat frame;
    frame = img.reshape(0, 1);
    
    int sockfd;
    const char *portno = "9622";
    struct sockaddr_in server_addr;
    struct addrinfo hints, *servinfo, *p;
    int status;
    try{
	memset(&hints, 0, sizeof(hints)); 
	hints.ai_family = AF_INET;
	hints.ai_socktype = SOCK_STREAM;
	int error = getaddrinfo("localhost", portno, &hints, &servinfo);
	if(error != 0){
	    cout << gai_strerror(error) << endl;
	    throw runtime_error("Could not get host info, exiting.");
	}

	for(p=servinfo; p != NULL; p = p->ai_next){
	    if((sockfd = socket(p->ai_family, p->ai_socktype, p->ai_protocol)) == -1){
		cout << "failed..." << endl;
		continue;
	    }
	    if(connect(sockfd, p->ai_addr, p->ai_addrlen) == -1){
		close(sockfd);
		continue;
	    }
	    break;
	}
	if(p == NULL)
	    throw runtime_error("Couldn't connect to server");

	struct img_packet packet;
	packet.cols = img.cols;
	packet.rows = img.rows;

	write(sockfd, &packet, sizeof(packet)); 

	int data_size = frame.total()*frame.elemSize();
	send(sockfd, frame.data, data_size, 0);

	int c = read(sockfd, &status, sizeof(int));
	close(sockfd);
    }catch (exception const& e){
	sleep(2);
	return processImage(img);
    }
    return status;
}
