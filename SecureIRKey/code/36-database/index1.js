// Modules
var level = require('level')
var express = require('express');
var app = require('express')();
var http = require('http').Server(app);
var io = require('socket.io')(http);
var fs = require('fs');
var parse = require('csv-parse');

// Create or open the underlying LevelDB store
var db = level('./mydb', {valueEncoding: 'json'});

var inputFile = 'smoke.csv';
var dataIn = {};

// Points to index.html to serve webpage
app.get('/', function(req, res){
  res.sendFile(__dirname + '/index1.html');
});


// Function to stream from database
function readDB(arg) {
  db.createReadStream()
    .on('data', function (data) {
      //console.log(data.key, '=', data.value);
      dataIn = {[data.key]: data.value};

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
http.listen(8080, function() {
  console.log('listening on *:8080');
});

var parser = parse({delimiter: ','}, function (err, data) {
    var i = 0;
    data.forEach(function(line) {
      var key = i;
      var value = [{time: line[0], id: line[1], smoke: line[2], temp: line[3]}];
      db.put(key, value, function (err) {
        if (err) return console.log('Ooops!', err); // some kind of I/O error
      })
      i++;
    });
});

// read the inputFile, feed the contents to the parser
fs.createReadStream(inputFile).pipe(parser);
