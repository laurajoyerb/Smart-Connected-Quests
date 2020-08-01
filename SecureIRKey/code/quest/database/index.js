// Modules
var level = require('level')
var express = require('express');
var app = require('express')();
var http = require('http').Server(app);
var io = require('socket.io')(http);

// Create or open the underlying LevelDB store
var db = level('./mydb', {valueEncoding: 'json'});

var dataIn = {};

// Points to index.html to serve webpage
app.get('/', function(req, res){
  res.sendFile(__dirname + '/index.html');
});


// Function to stream from database
function readDB(arg) {
  db.createReadStream()
    .on('data', function (data) {
      console.log(data.key, '=', data.value);
      dataIn = {[data.key]: data.value};

      //io.emit('message', dataIn);
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
io.on('connection', function(socket){
  console.log('a user connected');

  socket.on('disconnect', function(){
    console.log('user disconnected');
  });
});

// Pulls data from SerialPort
const SerialPort = require('serialport')
const Readline = require('@serialport/parser-readline')
const port = new SerialPort('/dev/cu.SLAB_USBtoUART34', { // change port depending on computer
  baudRate: 115200
});

const parser = port.pipe(new Readline({ delimiter: '\r\n' }))
parser.on('data', readSerialData)

var index = 0;

function readSerialData(data) {
  data = data.split(" ");
  console.log(data);
  var key = index;

  var passcode = data[2];
  var locationValue = "N/A";
  var statusValue = "Failed";
  var personValue = "N/A";

  // Passing fob_ID, hub_ID, code to get :
  // fob_ID, hub_ID, person, time, location, status

  // Checks passcode and assigns location based on hub_ID
  switch (data[1])
  {
    case "1":
      locationValue = "Tokyo, Japan";
      if (passcode == "3141")
      {
        statusValue = "Succeeded";
      }
      break;
    case "2":
      locationValue = "Buenos Aires, Argentina";
      if (passcode == "1234")
      {
        statusValue = "Succeeded";
      }
      break;
    case "3":
      locationValue = "Bora Bora, Tahiti";
      if (passcode == "4321")
      {
        statusValue = "Succeeded";
      }
      break;
    default:
      console.log("failed switch statement");
      break;
  }

  // Assigns person based on fob_ID
  if (data[0] == "1") {
    personValue = "Laura Joy";
  } else if (data[0] == "2") {
    personValue = "Erin";
  } else if (data[0] == "3") {
    personValue = "Yuting";
  }

  // Gets current time
  var today = new Date();
  var timeValue = today.getHours() + ":" + today.getMinutes() + ":" + today.getSeconds();

  var value = [{fob_ID: data[0], hub_ID: data[1], person: personValue, time: timeValue, location: locationValue, status: statusValue}];
  db.put(key, value, function (err) {
    if (err) return console.log('Ooops!', err); // some kind of I/O error
    console.log("Wrote to database");
  })
  index++;
  // Call function to stream database data
  readDB();

}


// Listening on localhost:8080
http.listen(8080, function() {
  console.log('listening on *:8080');
});
