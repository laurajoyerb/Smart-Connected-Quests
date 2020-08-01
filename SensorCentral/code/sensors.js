var app = require('express')();
var http = require('http').Server(app);

function toogleDataSeries(e){
	if (typeof(e.dataSeries.visible) === "undefined" || e.dataSeries.visible) {
		e.dataSeries.visible = false;
	} else{
		e.dataSeries.visible = true;
	}
	chart.render();
}

var ir = [];
var ultra = [];
var battery = [];
var therm = [];

var chartOptions = {
	title: {
		text: "Sensor Central"
	},
	axisX: {
		ValueFormatString: "#####",
		interval: 1
	},
	axisY: [ {
		title: "Distance",
		suffix: "m"
	},
	{
		title: "Temperature",
		suffix: "ºC"
	}],
	axisY2: {
		title: "Voltage",
		suffix: "mV"
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
		type:"line",
		name: "IR Range",
		showInLegend: true,
		markerSize: 0,
		yValueFormatString: "###0.00m",
		xValueFormatString: "##0s",
		axisYIndex: 0,
		dataPoints: ir
	},
	{
		type: "line",
		name: "Ultrasonic",
		showInLegend: true,
		markerSize: 0,
		yValueFormatString: "###0.00m",
		xValueFormatString: "##0s",
		axisYIndex: 0,
		dataPoints: ultra
	},
	{
		type: "line",
		axisYType: "secondary",
		name: "Battery",
		showInLegend: true,
		markerSize: 0,
		yValueFormatString: "###0.00mV",
		xValueFormatString: "##0s",
		dataPoints: battery
	},
	{
		type: "line",
		name: "Thermistor",
		showInLegend: true,
		markerSize: 0,
		yValueFormatString: "###0.0ºC",
		xValueFormatString: "##0s",
		axisYIndex: 1,
		dataPoints: therm
	}]
};

var io = require('socket.io')(http);

app.get('/', function(req, res){
  res.sendFile(__dirname + '/sensors.html');
  app.get('/data', function(req, res) {
    res.sendFile(__dirname + '/sensors.txt');
  });
});

setInterval(function(){
    io.emit('dataMsg', chartOptions);
}, 1000);

io.on('connection', function(socket){
  io.emit('dataMsg', chartOptions);

  socket.on('disconnect', function(){
    console.log('user disconnected');
  });
});

const SerialPort = require('serialport')
const Readline = require('@serialport/parser-readline')
const port = new SerialPort('/dev/cu.SLAB_USBtoUART', {
  baudRate: 115200
});

const parser = port.pipe(new Readline({ delimiter: '\r\n' }))
parser.on('data', readSerialData)

var starttime = null;

function readSerialData(data) {
	data = data.split(" ");
	time = parseInt(data[0], 10);
	if (starttime == null) {
		starttime = time;
	}
	time -= starttime;

	ir.push({
		x: time,
		y: parseFloat(data[1])
	});
	ultra.push({
		x: time,
		y: parseFloat(data[2])
	});
	battery.push({
		x: time,
		y: parseFloat(data[3])
	});
	therm.push({
		x: time,
		y: parseFloat(data[4])
	});

	if (ir.length > 30) {
		ir.shift();
		ultra.shift();
		battery.shift();
		therm.shift();
	}

}

http.listen(3000, function(){
  console.log('listening on *:3000');
});
