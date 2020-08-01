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

var sensor1 = [];
var sensor2 = [];
var sensor3 = [];
var sensor4 = [];
var sensor5 = [];
var smoke1 = [];
var smoke2 = [];
var smoke3 = [];
var smoke4 = [];
var smoke5 = [];

var index;
var timeData;
var idData;
var smokeData;
var tempData;

// Points to index.html to serve webpage
app.get('/', function(req, res){
  res.sendFile(__dirname + '/index.html');
});

// -------------------Chart stuff-------------------

function toogleDataSeries(e) {
	if (typeof (e.dataSeries.visible) === "undefined" || e.dataSeries.visible) {
		e.dataSeries.visible = false;
	} else {
		e.dataSeries.visible = true;
	}
	chart.render();
}

// Chart for sensor temperature data
var sensorChart = {
	title: {
		text: "Temperature Data"
	},
	axisX: {
		title: "Time"
	},
	axisY: {
		title: "Temperature"
	},
	toolTip: {
		shared: true
	},
	legend: {
		cursor: "pointer",
		verticalAlign: "top",
		horizontalAlign: "center",
		dockInsidePlotArea: true,
		itemclick: toogleDataSeries
	},
	data: [{
		type: "line",
		name: "Sensor 1",
		showInLegend: true,
		markerSize: 0,
		dataPoints: sensor1
	},
	{
    type: "line",
		name: "Sensor 2",
		showInLegend: true,
		markerSize: 0,
		dataPoints: sensor2
	},
	{
    type: "line",
		name: "Sensor 3",
		showInLegend: true,
		markerSize: 0,
		dataPoints: sensor3
	},
  {
    type: "line",
		name: "Sensor 4",
		showInLegend: true,
		markerSize: 0,
		dataPoints: sensor4
	},
  {
    type: "line",
		name: "Sensor 5",
		showInLegend: true,
		markerSize: 0,
		dataPoints: sensor5
	}]
};


// Function to stream from database
function readDB(arg) {
  db.createReadStream()
    .on('data', function (data) {
      //console.log(data.key, '=', data.value);
      dataIn = {[data.key]: data.value};

      // Take database data and put it into structures for chart
      index = Object.keys(dataIn);
      timeData = dataIn[Object.keys(dataIn)][0].time;
			idData = dataIn[Object.keys(dataIn)][0].id;
			smokeData = dataIn[Object.keys(dataIn)][0].smoke;
			tempData = dataIn[Object.keys(dataIn)][0].temp;

      if (sensor1.length < 8)
			{
        if (idData == 1){
          //console.log("Case 1");
          sensor1.push({ x: timeData, y: tempData});
          smoke1.push({ x: timeData, y: smokeData});
        } else if (idData == 2){
          sensor2.push({ x: timeData, y: tempData});
          smoke2.push({ x: timeData, y: smokeData});
          //console.log("Case 2");
        } else if (idData == 3){
          sensor3.push({ x: timeData, y: tempData});
          smoke3.push({ x: timeData, y: smokeData});
          //console.log("Case 3");
        } else if (idData == 4){
          //console.log("Case 4");
          sensor4.push({ x: timeData, y: tempData});
          smoke4.push({ x: timeData, y: smokeData});
        } else if (idData == 5){
          //console.log("Case 5");
          sensor5.push({ x: timeData, y: tempData});
          smoke5.push({ x: timeData, y: smokeData});
        }
      }
      console.log(sensorChart.data[0].dataPoints);
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


io.on('connection', function (socket) {
  io.emit('sensorData', sensorChart)
});

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
