var app = require('express')();
var http = require('http').Server(app);

// Required module
var dgram = require('dgram');

// Port and IP
var PORT = 8080;
var HOST = '192.168.1.127'; // when connected to Router Group_18

// Create socket
var server = dgram.createSocket('udp4');

// Create server
server.on('listening', function () {
	var address = server.address();
	console.log('UDP Server listening on ' + address.address + ":" + address.port);
});

var msgBack = '0';

// On connection, print out received message
server.on('message', function (msg, remote) {

	console.log(remote.address + ':' + remote.port + ' - ' + msg);

	// Send Ok acknowledgement
	server.send(msgBack, remote.port, remote.address, function (error) {
		if (error) {
			console.log('MEH!');
		}
		else {
			console.log('Sent: ' + msgBack);
		}
	});

});



// Bind server to port and IP
server.bind(PORT, HOST);
//------------------------------------------------------------------

var io = require('socket.io')(http);

app.get('/', function (req, res) {
	res.sendFile(__dirname + '/control.html');
});

io.on('connection', function (socket) {

  // Sending on/off dataPoints
  socket.on('crawlSet', function (on) {
		if (on == true) {
			msgBack = '1';
		}
    else {
	  msgBack = '0';
    }
	});

	socket.on('disconnect', function () {
		console.log('user disconnected');
	});
});




http.listen(8080, function () {
	console.log('listening on *:8080');
});
