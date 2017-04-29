"use strict";

/** */

var maxValue = 180;
var ongoingTouches = {}
//var sock = new WebSocket("ws://"+ location.hostname + ":81");
//var sock = {};
var sock = new WebSocket("ws://192.168.1.19:81");
var buttons = ["A", "B", "C", "D"];
var joys = ["L", "R"];

function startup() {
	var el = document.getElementsByTagName("body")[0];
	el.addEventListener("touchstart", handleStart, false);
	el.addEventListener("touchend", handleEnd, false);
	el.addEventListener("touchcancel", handleEnd, false); //handle the same as finger up
	el.addEventListener("touchmove", handleMove, false);
	sock.onopen = sockOpen; 
	sock.onmessage = sockMessage;
	
	buttons.forEach(function(b) { $("#" + b).on("click", handleClickEv); });
	$("#pidSet").on("click", function(e){
		e.preventDefault();
		var p = $("#inP").val();
		var i = $("#inI").val();
		var d = $("#inD").val();
		sock.send("P" + p + "," + i + "," + d)
	});
}

function sockOpen(e) {
	console.log("Socket connected");
	sock.send("HI");
}

function sockMessage(e) {
	console.log("Socket message: ");
	console.log(e.data);
	if (e.data.charAt(0) == "G") { //gauge data
		var id = "#" + e.data.substr(0, 2);
		var value = e.data.substr(2);
		$(id + ">p").css("width", value + "%");
	} else if (e.data.charAt(0) == "P") {
		var pid = e.data.substr(1).split(",");
		$("#inP").val(pid[0]);
		$("#inI").val(pid[1]);
		$("#inD").val(pid[2]);
	}
}

function handleStart(e) {
	e.preventDefault();
	var touches = e.changedTouches;
	for (var i = 0; i < touches.length; i++) {
		var t = touches[i];
		var tt = t.target;
		if (tt.nodeName == 'BUTTON' && joys.indexOf(tt.id) > -1) {
			ongoingTouches[t.identifier] = [ t.pageX, t.pageY, 0, 0 ];
		}
		if (tt.nodeName == 'BUTTON' && buttons.indexOf(tt.id) > -1) {
			handleClick(tt);
		}
	}
}

function handleEnd(e) {
	e.preventDefault();
	var touches = e.changedTouches;
	for (var i = 0; i < touches.length; i++) {
		var t = touches[i];
		if (ongoingTouches[t.identifier] != undefined) {
			delete(ongoingTouches[t.identifier]);
			t.target.setAttribute("style", "top:0");
			var dir = t.target.getAttribute("id");
			sock.send(dir + "0,0");
		}
	}
}

function handleMove(e) {
	e.preventDefault();
	var touches = e.changedTouches;
	for (var i = 0; i < touches.length; i++) {
		var t = touches[i];
		if (ongoingTouches[t.identifier] != undefined) {
			var changed = false;
			var touch = ongoingTouches[t.identifier];
			var deltaX = t.pageX - touch[0];
			if (deltaX > maxValue) { deltaX = maxValue; }
			if (deltaX < -maxValue) { deltaX = -maxValue; }
			if (deltaX != touch[2]) {
				ongoingTouches[t.identifier][2] = deltaX;
				changed = true;
			}
			
			var deltaY = t.pageY - touch[1];
			if (deltaY > maxValue) { deltaY = maxValue; }
			if (deltaY < -maxValue) { deltaY = -maxValue; }
			if (deltaY != touch[3]) {
				ongoingTouches[t.identifier][3] = deltaY;
				changed = true;
			}
			
			if (changed) {
				t.target.setAttribute("style", "top:" + deltaY + "px;left:" + deltaX + "px");
				var dir = t.target.getAttribute("id");
				var message = dir + deltaX + "," + deltaY;
				console.log("sending message:"); console.log(message);
				sock.send(message);
			}
		}
	}
}

function handleClickEv(e) {
	handleClick(e.target);
}

function handleClick(tt) {
	var id = tt.id;
	console.log("sending message:"); console.log(id);
	sock.send(id);
}

startup();
