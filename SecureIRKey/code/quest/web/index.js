// Modules
var level = require('level');
var express = require('express');
var app = require('express')();
var http = require('http').Server(app);
var io = require('socket.io')(http);
var dgram = require('dgram');

// Create or open the underlying LevelDB store
var db = level('./mydb', { valueEncoding: 'json' });

// Port and IP
var PORT = 8080;
var HOST = '192.168.1.127';

// Create socket
var server = dgram.createSocket('udp4');

// Create server
server.on('listening', function () {
  var address = server.address();
  console.log('UDP Server listening on ' + address.address + ":" + address.port);
});

var msgBack = '0';
var msgString;
var msgReceived;
var keystatus = 0;
var blinkindex = 0;
var blink_led = '-1';

// On connection, print out received message
server.on('message', function (msg, remote) {
  console.log(remote.address + ':' + remote.port + ' - ' + msg);

  msgString = msg.toString();
  msgReceived = msgString.split(" ");

  switch (msgReceived[0]) {
    case "1":
      if (keystatus == 11) {
        blinkindex = 10;
        blink_led = '1';
        keystatus = 0;
      }
      break;
    case "2":
      if ((keystatus == 21) && (blinkindex == 0)) {
        blinkindex = 10;
        blink_led = '2';
        keystatus = 0;
      }
      break;
    case "3":
      if (keystatus == 31) {
        blinkindex = 10;
        blink_led = '3';
        keystatus = 0;
      }
      break;
    case "4":
      readSerialData(msgString.substring(2, msgString.length));
      break;
    case "0":
      break;
    default:
      msgBack = '0';
      break;
  }


  if ((blinkindex > 0) && ((msgReceived[0] == blink_led) || msgReceived[0] == '0')) {
    msgBack = '1';
    blinkindex--;
  } else {
    msgBack = '0';
    if (blinkindex <= 0)
      blink_led = '-1';
  }

  // Send Ok acknowledgement
  server.send(msgBack, remote.port, remote.address, function (error) {
    if (error) {
      console.log('MEH!');
    }
    else {
      console.log('Sent:' + msgBack);
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

  socket.on('disconnect', function () {
    console.log('user disconnected');
  });
});


var index = 0;

function readSerialData(data) {
  data = data.split(" ");
  console.log(data);
  var key = index;

  var passcode = data[2];
  var locationValue = "N/A";
  var statusValue = "ACCESS DENIED";
  var personValue = "N/A";
  keystatus = 0;

  // Passing fob_ID, hub_ID, code to get :
  // fob_ID, hub_ID, person, time, location, status

  // Assigns location based on hub_ID
  switch (data[1]) {
    case "1":
      locationValue = "Tokyo, Japan";
      break;
    case "2":
      locationValue = "Buenos Aires, Argentina";
      break;
    case "3":
      locationValue = "Bora Bora, Tahiti";
      break;
    default:
      console.log("failed switch statement");
      break;
  }


  // Assigns person based on fob_ID and check passcode
  if (data[0] == "1") {
    personValue = "Laura Joy";
    if (passcode == '4321') {
      statusValue = "ACCESS GRANTED";
      keystatus = 11;
    } else {
      statusValue = "ACCESS DENIED";
      keystatus = 0;
    }
  } else if (data[0] == "2") {
    personValue = "Erin";
    if (passcode == '1234') {
      statusValue = "ACCESS GRANTED";
      keystatus = 21;
    } else {
      statusValue = "ACCESS DENIED";
      keystatus = 0;
    }
  } else if (data[0] == "3") {
    personValue = "Yuting";
    if (passcode == '3141') {
      statusValue = "ACCESS GRANTED";
      keystatus = 31;
    } else {
      statusValue = "ACCESS DENIED";
      keystatus = 0;
    }
  }

  // Gets current time
  var today = new Date();
  var timeValue = today.getHours() + ":" + today.getMinutes() + ":" + today.getSeconds();

  var value = [{ fob_ID: data[0], hub_ID: data[1], person: personValue, time: timeValue, location: locationValue, status: statusValue }];
  db.put(key, value, function (err) {
    if (err) return console.log('Ooops!', err); // some kind of I/O error
    console.log("Wrote to database");
  })
  index++;

  // Call function to stream database data
  readDB();

}


// Listening on localhost:8080
http.listen(8080, function () {
  console.log('listening on *:8080');
});
