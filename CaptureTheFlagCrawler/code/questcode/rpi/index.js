
// Modules
var level = require('level');
var express = require('express');
var app = require('express')();
var http = require('http').Server(app);
var io = require('socket.io')(http);
var dgram = require('dgram');
//const jsQR = require('jsqr');

// Type: sudo service motion start to start webcam feed

// Create or open the underlying LevelDB store
var db = level('./mydb', { valueEncoding: 'json' });


// Port and IP
var PORT = 8080;
var HOST = '192.168.1.106';

// Create socket
var server = dgram.createSocket('udp4');

// Create server
server.on('listening', function () {
  var address = server.address();
  console.log('UDP Server listening on ' + address.address + ":" + address.port);
});


// msgback = 'manual stop forward angle'
var msgback = '0 1 1 54';
//var msgto = '1 0.1 0 0 30';


var manual = 0;   // auto
var stop = 1;     // stop
var forward = 1;  // go forward
var angle = 54;   // straight


// On connection, print out received message
server.on('message', function (msg, remote) {
  console.log(remote.address + ':' + remote.port +' - ' + msg);

  readSerialData(msg);

  // Send Ok acknowledgement
  server.send(msgback,remote.port,remote.address,function(error){
    if(error){
      console.log('MEH!');
    }
    else{
      console.log('Sent: ' + msgback);
      msgback = manual.toString() + ' ' + stop.toString() + ' ' + forward.toString() + ' ' + angle.toString() + ' manual stop forward angle';
    }
  });

});

// Bind server to port and IP
server.bind(PORT, HOST);

var dataIn = {};

// Points to index.html to serve webpage
app.get('/', function (req, res) {
  res.sendFile(__dirname + '/index.html');
});


// Function to stream from database
function readDB(arg) {
  db.createReadStream()
    .on('data', function (data) {
      console.log(data.key, '=', data.value);
      dataIn = { [data.key]: data.value };

      io.emit('sendData', dataIn);
    })
    .on('error', function (err) {
      console.log('Oh my!', err)
    })
    .on('close', function () {
      console.log('Stream closed')
    })
    .on('end', function () {
      console.log('Stream ended')
    })
}

// When a new client connects
io.on('connection', function (socket) {
  console.log('a user connected');
  //readSerialData(msgto);

  socket.on('toManual', function (manual_) {
    // Do something
    manual = manual_;
  });

  socket.on('moveForward', function (forward_) {
    // Do something
    forward = forward_;
  });

  socket.on('moveLeft', function (angle_) {
    // Do something
    angle = angle_;
  });

  socket.on('moveStop', function (stop_) {
    // Do something
    stop = stop_;
  });

  socket.on('disconnect', function () {
    console.log('user disconnected');
  });
});

var index = 0;
var ID = 0;
var speed = 0;
var hour = -1;
var min = -1;
var sec = -1;

var lastID = 0;

function readSerialData(msg) {

  // msg = "0 speed - - - -"
  //     = "ID speed hour min sec -" when encounters a beacon
  data = (msg.toString()).split(' ');
  console.log("Data start: " + data + "\n");

  ID = parseInt(data[0]);
  speed = data[1];
  hour = data[2];
  min = data[3];
  sec = data[4];

  if((ID > 0) && (ID != lastID)){

    lastID = ID;

    // Write to database
    var key = index;

    // Gets current time
    var today = new Date();
    var timeValue = today.getHours() + ":" + today.getMinutes() + ":" + today.getSeconds();

    var value = [{ID: ID, speed: speed, time: timeValue, hour: hour, min: min, sec: sec}];
    console.log("Value" + value + "\n");
    db.put(key, value, function (err) {
      if (err) return console.log('Ooops!', err); // some kind of I/O error
      console.log("Wrote to database");
    })
    index++;
  }

  // Call function to stream database data
  readDB();

  if(ID == 3){
    manual = 1;
  }

}

setInterval(function () {
	io.emit('speedData', speed);
}, 1000);

// Listening on localhost:8080
http.listen(8080, function () {
  console.log('listening on *:8080');
});
