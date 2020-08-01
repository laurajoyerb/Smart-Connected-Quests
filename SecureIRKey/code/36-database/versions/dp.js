// Modules
var level = require('level')
var express = require('express');
var app = require('express')();
var http = require('http').Server(app);
var io = require('socket.io')(http);

// Create or open the underlying LevelDB store
var db = level('./mydb', {valueEncoding: 'json'});

// Random number function -- this is a helper function to generate dummy data
function getRndInteger(min, max) {
    return Math.floor(Math.random() * (max - min + 1) ) + min;
}

// Points to index.html to serve webpage
app.get('/', function(req, res){
  res.sendFile(__dirname + '/dp.html');
});

// Function to stream from database
function readDB(arg) {
  db.createReadStream()
    .on('data', function (data) {
      console.log(data.key, '=', data.value)
      // Parsed the data into a structure but don't have to ...
      var dataIn = {[data.key]: data.value};
      // Stream data to client
      io.emit('message', dataIn);
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
var clientConnected = 0; // this is just to ensure no new data is recorded during streaming
io.on('connection', function(socket){
  console.log('a user connected');
  clientConnected = 0;

  // Call function to stream database data
  readDB();
  clientConnected = 1;
  socket.on('disconnect', function(){
    console.log('user disconnected');
  });
});

// Listening on localhost:3000
http.listen(3000, function() {
  console.log('listening on *:3000');
});

// Every 15 seconds, write random information
function intervalFunc() {
  if (clientConnected == 1) {

    // Get current time
    var date = Date.now();

    // Fill in data structure
    var value = [{id: 1, temp: getRndInteger(30,80)}];

    // Put structure into database based on key == date, and value
    db.put([date], value, function (err) {
      if (err) return console.log('Ooops!', err) // some kind of I/O error
    })

    // Parse data to send to client
    var msg = {[date]: value};

    // Send to client
    io.emit('message', msg);

    // Log to console
    console.log(Object.keys(msg));
  }
}
// Do every 1500 ms
setInterval(intervalFunc, 1500);
