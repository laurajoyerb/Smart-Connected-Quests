var app = require('express')();
var http = require('http').Server(app);

// Combined Part----------------------------------------------------------------------------------
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

var msgBack = 'Ok!';

// On connection, print out received message
server.on('message', function (msg, remote) {

	readData(msg.toString());
	console.log(remote.address + ':' + remote.port + ' - ' + msg);

	// Send Ok acknowledgement
	server.send(msgBack, remote.port, remote.address, function (error) {
		if (error) {
			console.log('MEH!');
		}
		else {
			console.log('Sent: ' + msgBack);
			msgBack = 'Ok!';
		}
	});

});

// Bind server to port and IP
server.bind(PORT, HOST);
//------------------------------------------------------------------

function toogleDataSeries(e) {
	if (typeof (e.dataSeries.visible) === "undefined" || e.dataSeries.visible) {
		e.dataSeries.visible = false;
	} else {
		e.dataSeries.visible = true;
	}
	chart.render();
}

var accel_X = [];
var accel_Y = [];
var accel_Z = [];
var temp = [];
var battery = [];
var steps = 0;
var time_left = 20;
// water break

var accelChart = {
	title: {
		text: "Accelerometer"
	},
	axisX: {
		valueFormatString: "###"
	},
	axisY2: {
		title: "Acceleration (G's)",
		valueFormatString: "#.0"
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
		axisYType: "secondary",
		name: "X Acceleration",
		showInLegend: true,
		markerSize: 0,
		yValueFormatString: "#.00",
		dataPoints: accel_X
	},
	{
		type: "line",
		axisYType: "secondary",
		name: "Y Acceleration",
		showInLegend: true,
		markerSize: 0,
		yValueFormatString: "#.00",
		dataPoints: accel_Y
	},
	{
		type: "line",
		axisYType: "secondary",
		name: "Z Acceleration",
		showInLegend: true,
		markerSize: 0,
		yValueFormatString: "#.00",
		dataPoints: accel_Z
	}]
};

var tempChart = {
	title: {
		text: "Body Temperature"
	},
	axisX: {
		valueFormatString: "###"
	},
	axisY2: {
		title: "Temperature",
		valueFormatString: "##ºC"
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
		axisYType: "secondary",
		name: "Body Temperature",
		color: "red",
		showInLegend: true,
		markerSize: 0,
		yValueFormatString: "#.0",
		dataPoints: temp
	}]
};

var batteryChart = {
	title: {
		text: "Battery Level"
	},
	axisX: {
		valueFormatString: "###"
	},
	axisY2: {
		title: "Voltage",
		valueFormatString: "##V"
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
		axisYType: "secondary",
		name: "Voltage",
		color: "green",
		showInLegend: true,
		markerSize: 0,
		yValueFormatString: "#.0",
		dataPoints: battery
	}]
};

var io = require('socket.io')(http);

app.get('/', function (req, res) {
	res.sendFile(__dirname + '/sensors.html');
});

setInterval(function () {
	io.emit('accelData', accelChart);
	io.emit('tempData', tempChart);
	io.emit('batteryData', batteryChart);
	io.emit('stepsData', steps);
	io.emit('timeData', time_left);
}, 1000);

io.on('connection', function (socket) {
	io.emit('accelData', accelChart);
	io.emit('tempData', tempChart);
	io.emit('batteryData', batteryChart);
	io.emit('stepsData', steps);
	io.emit('timeData', time);

	socket.on('alarmTime', function (alarm) {
		if (alarm != null) {
			msgBack = alarm;
			console.log('New alarm: ' + msgBack);
		}
	});

	socket.on('disconnect', function () {
		console.log('user disconnected');
	});
});

var starttime = null;
var time;
var time_left;

function readData(data) {

	data = data.split(" ");
	time = parseInt(data[0], 10);
	if (starttime == null) {
		starttime = time;
	}
	time -= starttime;

	accel_X.push({
		x: time,
		y: parseFloat(data[4])
	});
	accel_Y.push({
		x: time,
		y: parseFloat(data[5])
	});
	accel_Z.push({
		x: time,
		y: parseFloat(data[6])
	});
	temp.push({
		x: time,
		y: parseFloat(data[1])
	});
	battery.push({
		x: time,
		y: parseFloat(data[3])
	});

	steps = parseInt(data[2], 10);
	time_left = parseInt(data[7], 10);

	if (accel_X.length > 40) {
		accel_X.shift();
		accel_Y.shift();
		accel_Z.shift();
	}

	if (temp.length > 40) {
		temp.shift();
	}

	if (battery.length > 40) {
		battery.shift();
	}

}

http.listen(8080, function () {
	console.log('listening on *:8080');
});
