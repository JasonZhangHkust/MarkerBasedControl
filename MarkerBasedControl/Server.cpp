#include "Server.h"

Server::Server(int PORT, bool BroadcastPublically) //Port = port to broadcast on. BroadcastPublically = false if server is not open to the public (people outside of your router), true = server is open to everyone (assumes that the port is properly forwarded on router settings)
{
	//Winsock Startup
	WSAData wsaData;
	WORD DllVersion = MAKEWORD(2, 1);
	memset(close, false, sizeof(close));
	for (int i = 0; i < 5; i++) {
		order.push_back("6680808000808099");
		orderNew.push_back("6680808000808099");
	}
	memset(flag, true, sizeof(flag));

	if (WSAStartup(DllVersion, &wsaData) != 0) //If WSAStartup returns anything other than 0, then that means an error has occured in the WinSock Startup.
	{
		MessageBoxA(NULL, "WinSock startup failed", "Error", MB_OK | MB_ICONERROR);
		exit(1);
	}

	if (BroadcastPublically) //If server is open to public
		addr.sin_addr.s_addr = htonl(INADDR_ANY);
	else //If server is only for our router
		addr.sin_addr.s_addr = inet_addr("192.168.1.101"); //192.168.1.101
	addr.sin_port = htons(PORT); //Port
	addr.sin_family = AF_INET; //IPv4 Socket

	sListen = socket(AF_INET, SOCK_STREAM, NULL); //Create socket to listen for new connections
	if (bind(sListen, (SOCKADDR*)&addr, sizeof(addr)) == SOCKET_ERROR) //Bind the address to the socket, if we fail to bind the address..
	{
		std::string ErrorMsg = "Failed to bind the address to our listening socket. Winsock Error:" + std::to_string(WSAGetLastError());
		MessageBoxA(NULL, ErrorMsg.c_str(), "Error", MB_OK | MB_ICONERROR);
		exit(1);
	}
	if (listen(sListen, SOMAXCONN) == SOCKET_ERROR) //Places sListen socket in a state in which it is listening for an incoming connection. Note:SOMAXCONN = Socket Oustanding Max Connections, if we fail to listen on listening socket...
	{
		std::string ErrorMsg = "Failed to listen on listening socket. Winsock Error:" + std::to_string(WSAGetLastError());
		MessageBoxA(NULL, ErrorMsg.c_str(), "Error", MB_OK | MB_ICONERROR);
		exit(1);
	}
	serverptr = this;
}

bool Server::ListenForNewConnection()
{
	SOCKET newConnection = accept(sListen, (SOCKADDR*)&addr, &addrlen); //Accept a new connection
	if (newConnection == 0) //If accepting the client connection failed
	{
		std::cout << "Failed to accept the client's connection." << std::endl;
		return false;
	}
	else //If client connection properly accepted
	{
		std::cout << "Client Connected! ID:" << TotalConnections << std::endl;
		Connections[TotalConnections] = newConnection; //Set socket in array to be the newest connection before creating the thread to handle this client's socket.
		h =CreateThread(NULL, NULL, (LPTHREAD_START_ROUTINE)ClientHandlerThread, (LPVOID)(TotalConnections), NULL, NULL); //Create Thread to handle this client. The index in the socket array for this thread is the value (i).
		std::string MOTD = "MOTD: Welcome! This is the message of the day!.";
		TotalConnections += 1; //Incremenent total # of clients that have connected
		return true;
	}
}

void Server::ClientHandlerThread(int ID) //ID = the index in the SOCKET Connections array
{

	while (serverptr!=nullptr&&!serverptr->close[ID])
	{
		if (serverptr->flag[ID]) {
			serverptr->string2char2(serverptr->order[ID]);
			int RetnCheck = send(serverptr->Connections[ID], (char*) serverptr->y, 8, NULL);
			if (RetnCheck == SOCKET_ERROR)
				//std::cout << std::hex << clientptr->order;
				break;
			Sleep(50);
		}
		else {
			serverptr->order[ID] = serverptr->orderNew[ID];
			serverptr->flag[ID] = true;
		}

	}
	std::cout << "Lost connection to client ID: " << ID << std::endl;
	closesocket(serverptr->Connections[ID]);
}
void Server::update(std::string s, int ID) {

	orderNew[ID] = s;
	flag[ID] = false;
}
void Server::string2char2(std::string order) {
	int hexsize = order.length();
	BYTE myChar[8];

	for (unsigned i = 0; i<hexsize; i += 2)
		myChar[i / 2] = strtol(order.substr(i, 2).c_str(), 0, 16);
	for (int i = 0; i < 8; i++) {
		y[i] = myChar[i];
	}
}